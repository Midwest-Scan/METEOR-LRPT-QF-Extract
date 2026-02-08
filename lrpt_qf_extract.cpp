#include <cstdint>
#include <fstream>
#include <iostream>
#include <unordered_map>
#include <vector>
#include <array>
#include <set>
#include <filesystem>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <stdexcept>

#include <turbojpeg.h>


// CADU/CCSDS framing constants
static constexpr size_t   CADU_SIZE = 1024;
static constexpr uint32_t ASM = 0x1ACFFC1D;

static constexpr size_t OFF_PRIMARY = 4;
static constexpr size_t OFF_CODING = OFF_PRIMARY + 6;
static constexpr size_t OFF_ACCESS = OFF_CODING + 2;
static constexpr size_t OFF_ZONE = OFF_ACCESS + 2;
static constexpr size_t ZONE_LEN = 882;
static constexpr uint8_t TARGET_VCID = 5;

// Segment-span guard
static constexpr uint32_t SEG_CNT = 20000;

// Maximums for sanity
static constexpr uint32_t MAX_PACKET_BYTES = 4096;
static constexpr size_t   MAX_CARRY_BYTES = 16384;
static constexpr uint32_t MAX_QF_SEGMENTS = ((SEG_CNT + 13) / 14) * 14; // ceil(SEG_CNT/14)*14 = 20006.

// QF extraction offsets
// payload[8]    : MCUN (1B)
// payload[11:14]: segment header (FF F0 QF)
static constexpr size_t MIN_PAYLOAD_FOR_QF = 14;
static constexpr size_t MCUN_OFF = 8;
static constexpr size_t SEG_HDR_OFF = 11;

// Output names for per-channel QF TIFFs
static constexpr const char* CH_OUT_NAMES[6] = {
    "MSU-MR-1_qf.tif",
    "MSU-MR-2_qf.tif",
    "MSU-MR-3_qf.tif",
    "MSU-MR-4_qf.tif",
    "MSU-MR-5_qf.tif",
    "MSU-MR-6_qf.tif",
};

static inline int apid_to_channel(uint16_t apid) {
    if (apid >= 64 && apid <= 69) return int(apid - 64);
    return -1;
}
static inline int apid_to_digit(uint16_t apid) {
    return int(apid - 63); // APID 64->1 ... 69->6
}

// Big-endian helpers
static inline uint16_t be16(const uint8_t* p) {
    return (uint16_t(p[0]) << 8) | uint16_t(p[1]);
}
static inline uint32_t be32(const uint8_t* p) {
    return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | uint32_t(p[3]);
}

// Fixed-range scaling for composite: preserve 0 as "missing"
// Map QF 20..100 -> 1..255
static inline uint8_t scale_qf_20_100_to_1_255(uint8_t qf) {
    if (qf == 0) return 0; // missing

    constexpr int QF_MIN = 20;
    constexpr int QF_MAX = 100;
    constexpr int SPAN = (QF_MAX - QF_MIN); // 80

    int v = int(qf);
    if (v < QF_MIN) v = QF_MIN;
    if (v > QF_MAX) v = QF_MAX;

    int x = v - QF_MIN; // 0..80

    // Map 0..80 -> 1..255 using rounding:
    // scaled = 1 + round(x * 254 / 80)
    int scaled = 1 + (x * 254 + (SPAN / 2)) / SPAN;

    if (scaled < 1) scaled = 1;
    if (scaled > 255) scaled = 255;
    return uint8_t(scaled);
}

// Minimal baseline TIFF writer (8-bit grayscale, uncompressed, single strip)
static void write_u16(std::ofstream& out, uint16_t v) {
    out.put(char(v & 0xFF));
    out.put(char((v >> 8) & 0xFF));
}
static void write_u32(std::ofstream& out, uint32_t v) {
    out.put(char(v & 0xFF));
    out.put(char((v >> 8) & 0xFF));
    out.put(char((v >> 16) & 0xFF));
    out.put(char((v >> 24) & 0xFF));
}
struct TiffIFDEntry {
    uint16_t tag;
    uint16_t type;
    uint32_t count;
    uint32_t value_or_offset;
};
static bool write_tiff_gray8(const std::filesystem::path& out_path, uint32_t width, uint32_t height, const std::vector<uint8_t>& pixels)
{
    if (width == 0 || height == 0) return false;
    if (pixels.size() != size_t(width) * size_t(height)) return false;

    std::ofstream out(out_path, std::ios::binary);
    if (!out) return false;

    // TIFF header (little-endian): "II", 42, IFD offset
    out.put('I'); out.put('I');
    write_u16(out, 42);

    const uint32_t strip_offset = 8;
    const uint32_t strip_byte_count = uint32_t(pixels.size());

    const uint32_t ifd_offset = strip_offset + strip_byte_count;
    write_u32(out, ifd_offset);

    // Pixel data
    out.write(reinterpret_cast<const char*>(pixels.data()), std::streamsize(pixels.size()));

    // IFD entries
    std::vector<TiffIFDEntry> entries;
    entries.reserve(10);
    entries.push_back({ 256, 4, 1, width });              // ImageWidth
    entries.push_back({ 257, 4, 1, height });             // ImageLength
    entries.push_back({ 258, 3, 1, 8 });                  // BitsPerSample
    entries.push_back({ 259, 3, 1, 1 });                  // Compression = none
    entries.push_back({ 262, 3, 1, 1 });                  // Photometric = BlackIsZero
    entries.push_back({ 273, 4, 1, strip_offset });       // StripOffsets
    entries.push_back({ 277, 3, 1, 1 });                  // SamplesPerPixel
    entries.push_back({ 278, 4, 1, height });             // RowsPerStrip
    entries.push_back({ 279, 4, 1, strip_byte_count });   // StripByteCounts
    entries.push_back({ 284, 3, 1, 1 });                  // PlanarConfiguration = chunky

    write_u16(out, uint16_t(entries.size()));
    for (const auto& e : entries) {
        write_u16(out, e.tag);
        write_u16(out, e.type);
        write_u32(out, e.count);
        write_u32(out, e.value_or_offset);
    }
    write_u32(out, 0); // next IFD = none

    return bool(out);
}

// Libjpeg-turbo writer
static bool write_jpeg_rgb_turbo(const std::filesystem::path& out_path, const std::vector<uint8_t>& rgb, int width, int height, int quality)
{
    if (width <= 0 || height <= 0) return false;
    if (rgb.size() != size_t(width) * size_t(height) * 3) return false;

    tjhandle handle = tjInitCompress();
    if (!handle) return false;

    unsigned char* jpegBuf = nullptr;
    unsigned long jpegSize = 0;

    const int pitch = width * 3;

    int rc = tjCompress2(handle, rgb.data(), width, pitch, height, TJPF_RGB, &jpegBuf, &jpegSize, TJSAMP_420, quality, TJFLAG_FASTDCT);

    tjDestroy(handle);

    if (rc != 0 || !jpegBuf || jpegSize == 0) {
        if (jpegBuf) tjFree(jpegBuf);
        return false;
    }

    std::ofstream out(out_path, std::ios::binary);
    if (!out) {
        tjFree(jpegBuf);
        return false;
    }
    out.write(reinterpret_cast<const char*>(jpegBuf), std::streamsize(jpegSize));
    tjFree(jpegBuf);
    return bool(out);
}

// QF reconstructor
struct ChannelStats {
    bool has_values = false;
    uint8_t minv = 255;
    uint8_t maxv = 0;
    double avg = 0.0;
};

struct MSUMRQFReconstructor {
    std::array<uint32_t, 6> firstSeg;
    std::array<uint32_t, 6> offset;
    std::array<uint32_t, 6> rollover;
    std::array<uint16_t, 6> lastSeq;
    std::array<uint32_t, 6> lastSeg;

    std::array<std::unordered_map<uint32_t, uint8_t>, 6> segQF;
    std::set<uint16_t> apids_seen;

    MSUMRQFReconstructor() {
        for (int i = 0; i < 6; i++) {
            firstSeg[i] = 0xFFFFFFFFu;
            offset[i] = 0xFFFFFFFFu;
            rollover[i] = 0;
            lastSeq[i] = 0;
            lastSeg[i] = 0;
        }
    }

    void store_segment(int ch, uint32_t seg_id, uint8_t qf) {
        auto& m = segQF[ch];
        auto it = m.find(seg_id);
        if (it != m.end()) {
            it->second = qf;
            return;
        }
        if (m.size() >= MAX_QF_SEGMENTS) return; // hard cap per channel
        m.emplace(seg_id, qf);
    }

    // Accept complete OR partial packet if there are enough bytes to reach QF location.
    void ingest_packet_partial_ok(const std::vector<uint8_t>& pkt) {
        if (pkt.size() < 6) return;

        const uint16_t w0 = be16(pkt.data());
        const bool shf = (((w0 >> 11) & 1) != 0);
        const uint16_t apid = (w0 & 0x07FF);

        const int ch = apid_to_channel(apid);
        if (ch < 0) return;
        if (!shf) return;

        apids_seen.insert(apid);

        const uint16_t w1 = be16(pkt.data() + 2);
        const uint16_t sequence = (w1 & 0x3FFF);

        const size_t payload_len = pkt.size() - 6;
        if (payload_len < MIN_PAYLOAD_FOR_QF) return;

        const uint8_t* payload = pkt.data() + 6;

        const uint8_t mcun = payload[MCUN_OFF];
        const uint8_t s0 = payload[SEG_HDR_OFF + 0];
        const uint8_t s1 = payload[SEG_HDR_OFF + 1];
        const uint8_t qf = payload[SEG_HDR_OFF + 2];

        if (!(s0 == 0xFF && s1 == 0xF0)) return;

        const uint32_t mcu_count = uint32_t(mcun / 14);
        if (mcu_count > 13) return;

        // Rollover detect (15% threshold at each end)
        if (lastSeq[ch] > sequence && lastSeq[ch] > 13926 && sequence < 2458)
            rollover[ch] += 16384;

        // Offset extrapolation
        if (offset[ch] == 0xFFFFFFFFu) {
            const uint32_t mcu_seq = uint32_t(sequence) + (mcu_count > uint32_t(sequence) ? 16384u : 0u) - mcu_count;
            offset[ch] = (mcu_seq + rollover[ch]) % 43u;
        }

        // ID mapping
        const uint32_t seg_id = ((uint32_t(sequence) + rollover[ch] - offset[ch]) / 43u) * 14u + mcu_count;

        const uint32_t new_first = (firstSeg[ch] > seg_id) ? seg_id : firstSeg[ch];
        const uint32_t new_last = (lastSeg[ch] < seg_id) ? seg_id : lastSeg[ch];
        if (new_last - new_first > SEG_CNT) return; // SEG_CNT guard

        firstSeg[ch] = new_first;
        lastSeg[ch] = new_last;
        lastSeq[ch] = sequence;

        store_segment(ch, seg_id, qf);
    }

    static inline uint32_t floor14(uint32_t v) { return v - (v % 14u); }

    std::pair<uint32_t, uint32_t> compute_bounds(int channel) const {
        uint32_t firstSeg_line = 0xFFFFFFFFu;
        uint32_t firstSeg_before = 0xFFFFFFFFu;
        uint32_t firstSeg_after = 0xFFFFFFFFu;

        uint32_t lastSeg_line = 0u;
        uint32_t lastSeg_before = 0u;
        uint32_t lastSeg_after = 0u;

        int channel_with_lowest_offset = 6;
        int channel_lowest_transmitted = 6;

        // Alignment info
        for (int i = 0; i < 6; i++) {
            if (offset[i] == 0xFFFFFFFFu) continue;
            if (channel_lowest_transmitted == 6) channel_lowest_transmitted = i;
            if (channel_with_lowest_offset == 6 || offset[i] < offset[channel_with_lowest_offset]) channel_with_lowest_offset = i;
        }

        if (channel_lowest_transmitted == 6 || channel_with_lowest_offset == 6) return { 0u, 0u };

        // First/last segment across channels
        for (int i = 0; i < 6; i++) {
            if (offset[i] == 0xFFFFFFFFu) continue;

            if (firstSeg[i] < firstSeg_line) firstSeg_line = firstSeg[i];
            if (lastSeg[i] > lastSeg_line) lastSeg_line = lastSeg[i];

            if (i < channel_with_lowest_offset) {
                if (firstSeg[i] < firstSeg_before) firstSeg_before = firstSeg[i];
                if (lastSeg[i] > lastSeg_before) lastSeg_before = lastSeg[i];
            }
            else {
                if (firstSeg[i] < firstSeg_after) firstSeg_after = firstSeg[i];
                if (lastSeg[i] > lastSeg_after) lastSeg_after = lastSeg[i];
            }
        }

        if (channel_lowest_transmitted != channel_with_lowest_offset) {
            const bool first_shift_direction = (floor14(firstSeg_before) >= floor14(firstSeg_after));
            const bool last_shift_direction = (floor14(lastSeg_before) < floor14(lastSeg_after));

            auto sub14_clamp0 = [](uint32_t v) -> uint32_t { return (v >= 14u) ? (v - 14u) : 0u; };

            if (channel < channel_with_lowest_offset) {
                if (first_shift_direction) firstSeg_line = sub14_clamp0(firstSeg_line);
                if (last_shift_direction) lastSeg_line = sub14_clamp0(lastSeg_line);
            }
            else {
                if (!first_shift_direction) firstSeg_line += 14u;
                if (!last_shift_direction) lastSeg_line += 14u;
            }
        }

        // One line after last line (exclusive upper bound)
        lastSeg_line += 14u;

        // If current channel has no data
        if (firstSeg[channel] == 0xFFFFFFFFu) firstSeg_line = 0u;
        if (lastSeg[channel] == 0u) lastSeg_line = 0u;

        // Align to boundaries
        if (firstSeg_line != 0u) firstSeg_line = floor14(firstSeg_line);
        if (lastSeg_line != 0u) lastSeg_line = floor14(lastSeg_line);

        return { firstSeg_line, lastSeg_line };
    }

    // Build 14xrows QF pixels + compute stats (ignoring zeros).
    bool build_qf_tif_for_channel(int ch, std::vector<uint8_t>& out_pixels, uint32_t& out_w, uint32_t& out_h, ChannelStats& out_stats) const
    {
        out_stats = {};
        if (ch < 0 || ch > 5) return false;
        if (offset[ch] == 0xFFFFFFFFu) return false;
        if (segQF[ch].empty()) return false;

        const auto [firstSeg_line, lastSeg_line] = compute_bounds(ch);
        if (lastSeg_line == 0u || lastSeg_line < firstSeg_line) return false;

        const uint32_t width = 14u;
        const uint32_t rows = (lastSeg_line - firstSeg_line) / 14u;
        if (rows == 0u) return false;

        if (rows > (MAX_QF_SEGMENTS / 14u)) return false;

        out_pixels.assign(size_t(width) * size_t(rows), 0u);

        const auto& m = segQF[ch];

        uint64_t sum = 0;
        uint32_t cnt = 0;
        uint8_t mn = 255, mx = 0;

        for (uint32_t r = 0; r < rows; r++) {
            const uint32_t x = firstSeg_line + r * 14u;
            for (uint32_t j = 0; j < 14u; j++) {
                const uint32_t id = x + j;
                uint8_t v = 0;
                auto it = m.find(id);
                if (it != m.end()) v = it->second;
                out_pixels[size_t(r) * width + j] = v;

                if (v != 0) {
                    mn = std::min(mn, v);
                    mx = std::max(mx, v);
                    sum += v;
                    cnt++;
                }
            }
        }

        out_w = width;
        out_h = rows;

        if (cnt > 0) {
            out_stats.has_values = true;
            out_stats.minv = mn;
            out_stats.maxv = mx;
            out_stats.avg = double(sum) / double(cnt);
        }
        return true;
    }
};

// CADU -> CCSDS reassembly
static void try_process_carry(MSUMRQFReconstructor& recon, std::vector<uint8_t>& carry, bool discarding)
{
    if (carry.size() < 6) return;

    const uint16_t w2 = be16(carry.data() + 4);
    const uint32_t expected_total = 6u + (uint32_t(w2) + 1u);

    const bool enough_for_qf = (carry.size() >= 6u + MIN_PAYLOAD_FOR_QF);

    if (expected_total < 6u || expected_total > MAX_PACKET_BYTES) {
        if (discarding && enough_for_qf) recon.ingest_packet_partial_ok(carry);
        return;
    }

    if (carry.size() == expected_total) {
        recon.ingest_packet_partial_ok(carry);
        return;
    }

    if (discarding && enough_for_qf) {
        recon.ingest_packet_partial_ok(carry);
        return;
    }
}

static void extract_from_cadu(const std::filesystem::path& cadu_path, MSUMRQFReconstructor& recon)
{
    std::ifstream f(cadu_path, std::ios::binary);
    if (!f) throw std::runtime_error("Failed to open input CADU");

    std::vector<uint8_t> frame(CADU_SIZE);
    std::vector<uint8_t> carry;
    carry.reserve(4096);

    while (true) {
        f.read(reinterpret_cast<char*>(frame.data()), std::streamsize(CADU_SIZE));
        if (!f) break;
        if (f.gcount() != std::streamsize(CADU_SIZE)) break;

        // ASM check
        if (be32(frame.data()) != ASM) continue;

        // VCID gate (only demux VCID 5)
        const uint16_t tf0 = be16(frame.data() + OFF_PRIMARY);
        const uint8_t vcid = uint8_t(tf0 & 0x3F);
        if (vcid != TARGET_VCID) continue;

        // Access header
        const uint16_t acc = be16(frame.data() + OFF_ACCESS);
        const uint16_t marker5 = (acc >> 11) & 0x1F;
        uint16_t fhp = acc & 0x7FF;
        bool header_present = (marker5 == 0);

        // Defensive FHP clamp
        if (fhp > ZONE_LEN) header_present = false;

        const uint8_t* zone = frame.data() + OFF_ZONE;

        // Append continuation bytes
        if (header_present) {
            if (fhp > 0 && !carry.empty()) {
                carry.insert(carry.end(), zone, zone + fhp);
            }
        }
        else {
            if (!carry.empty()) {
                carry.insert(carry.end(), zone, zone + ZONE_LEN);
            }
        }

        // Carry hard cap
        if (carry.size() > MAX_CARRY_BYTES) {
            try_process_carry(recon, carry, true);
            carry.clear();
        }

        // New packet start in this frame: if carry isn't complete, finalize partial (if possible) and reset.
        if (header_present && !carry.empty()) {
            if (carry.size() >= 6) {
                const uint16_t w2 = be16(carry.data() + 4);
                const uint32_t expected_total = 6u + (uint32_t(w2) + 1u);
                if (carry.size() != expected_total) {
                    try_process_carry(recon, carry, true);
                    carry.clear();
                }
            }
        }

        // Try finalize carry if complete/insane/overrun
        if (!carry.empty() && carry.size() >= 6) {
            const uint16_t w2 = be16(carry.data() + 4);
            const uint32_t expected_total = 6u + (uint32_t(w2) + 1u);

            if (expected_total < 6u || expected_total > MAX_PACKET_BYTES) {
                try_process_carry(recon, carry, true);
                carry.clear();
            }
            else if (carry.size() == expected_total) {
                recon.ingest_packet_partial_ok(carry);
                carry.clear();
            }
            else if (carry.size() > expected_total) {
                try_process_carry(recon, carry, true);
                carry.clear();
            }
        }

        // Parse new packet starts in this frame zone
        size_t off = header_present ? size_t(fhp) : ZONE_LEN;
        while (off < ZONE_LEN) {
            if (off + 6 > ZONE_LEN) {
                // Straddling header
                carry.insert(carry.end(), zone + off, zone + ZONE_LEN);
                if (carry.size() > MAX_CARRY_BYTES) {
                    try_process_carry(recon, carry, true);
                    carry.clear();
                }
                break;
            }

            const uint16_t w2 = be16(zone + off + 4);
            const uint32_t expected_total = 6u + (uint32_t(w2) + 1u);

            if (expected_total < 6u || expected_total > MAX_PACKET_BYTES) {
                off += 1; // resync
                continue;
            }

            carry.clear();
            carry.insert(carry.end(), zone + off, zone + off + 6);

            const size_t avail_after_hdr = ZONE_LEN - (off + 6);
            const size_t need_after_hdr = size_t(expected_total - 6u);
            const size_t take = std::min(avail_after_hdr, need_after_hdr);

            carry.insert(carry.end(), zone + off + 6, zone + off + 6 + take);
            off += 6 + take;

            if (carry.size() == expected_total) {
                recon.ingest_packet_partial_ok(carry);
                carry.clear();
                continue;
            }

            // Continues into next frame
            if (carry.size() > MAX_CARRY_BYTES) {
                try_process_carry(recon, carry, true);
                carry.clear();
            }
            break;
        }
    }

    // EOF: accept partial if enough
    if (!carry.empty()) {
        try_process_carry(recon, carry, true);
        carry.clear();
    }
}

// JSON writer
static bool write_quality_json(const std::filesystem::path& out_path, const std::vector<std::pair<uint16_t, ChannelStats>>& stats_by_apid)
{
    std::ofstream out(out_path, std::ios::binary);
    if (!out) return false;

    out << "{\n";
    for (size_t i = 0; i < stats_by_apid.size(); i++) {
        const auto apid = stats_by_apid[i].first;
        const auto st = stats_by_apid[i].second;

        out << "  \"" << apid << "\": { ";

        if (st.has_values)
            out << "\"min\": " << int(st.minv) << ", \"max\": " << int(st.maxv) << ", \"avg\": " << std::fixed << std::setprecision(6) << st.avg;
        else
            out << "\"min\": null, \"max\": null, \"avg\": null";

        out << " }";
        if (i + 1 != stats_by_apid.size()) out << ",";
        out << "\n";
    }
    out << "}\n";
    return bool(out);
}

// Composite generator: 3 smallest APIDs -> RGB, expanded to 1568 x (rows*8)
// Scaling: QF 20..100 -> 1..255, missing stays 0.
static bool build_rgb_composite_from_segments(const MSUMRQFReconstructor& recon, const std::array<int, 3>& chs, const std::array<uint16_t, 3>& apids, std::vector<uint8_t>& out_rgb, int& out_w, int& out_h, std::string& out_digits)
{
    // Use union bounds across the three channels, so composite covers everything received.
    uint32_t min_first = 0xFFFFFFFFu;
    uint32_t max_last = 0u;

    for (int k = 0; k < 3; k++) {
        const int ch = chs[k];
        const auto b = recon.compute_bounds(ch);
        if (b.second == 0u) return false;
        min_first = std::min(min_first, b.first);
        max_last = std::max(max_last, b.second);
    }
    if (min_first == 0xFFFFFFFFu || max_last == 0u || max_last < min_first) return false;

    const uint32_t seg_rows = (max_last - min_first) / 14u;
    if (seg_rows == 0u) return false;

    if (seg_rows > (MAX_QF_SEGMENTS / 14u)) return false;

    // Expanded dimensions: each segment becomes 112x8 block
    const int width_px = 1568;
    const int height_px = int(seg_rows * 8u);

    constexpr int seg_w = 112;
    constexpr int seg_h = 8;

    out_w = width_px;
    out_h = height_px;
    out_rgb.assign(size_t(out_w) * size_t(out_h) * 3, 0u);

    // Digits for filename: APID 64->1 ... 69->6
    out_digits.clear();
    for (int k = 0; k < 3; k++) {
        out_digits.push_back(char('0' + apid_to_digit(apids[k])));
    }

    const auto& mapR = recon.segQF[chs[0]];
    const auto& mapG = recon.segQF[chs[1]];
    const auto& mapB = recon.segQF[chs[2]];

    for (uint32_t r = 0; r < seg_rows; r++) {
        const uint32_t base_id = min_first + r * 14u;

        for (uint32_t j = 0; j < 14u; j++) {
            const uint32_t id = base_id + j;

            uint8_t qfr = 0, qfg = 0, qfb = 0;

            auto itR = mapR.find(id); if (itR != mapR.end()) qfr = itR->second;
            auto itG = mapG.find(id); if (itG != mapG.end()) qfg = itG->second;
            auto itB = mapB.find(id); if (itB != mapB.end()) qfb = itB->second;

            // Apply fixed-range scaling for the composite
            qfr = scale_qf_20_100_to_1_255(qfr);
            qfg = scale_qf_20_100_to_1_255(qfg);
            qfb = scale_qf_20_100_to_1_255(qfb);

            const int x0 = int(j) * seg_w;
            const int y0 = int(r) * seg_h;

            for (int yy = 0; yy < seg_h; yy++) {
                uint8_t* rowp = &out_rgb[(size_t(y0 + yy) * size_t(out_w) + size_t(x0)) * 3];
                for (int xx = 0; xx < seg_w; xx++) {
                    rowp[0] = qfr;
                    rowp[1] = qfg;
                    rowp[2] = qfb;
                    rowp += 3;
                }
            }
        }
    }

    return true;
}

// Main
static void usage(const char* argv0) {
    std::cerr << "Usage: " << argv0 << " <input.cadu>\n";
}

int main(int argc, char** argv) {
    if (argc != 2) {
        usage(argv[0]);
        return 2;
    }

    const std::filesystem::path cadu_path = argv[1];
    if (!std::filesystem::exists(cadu_path)) {
        std::cerr << "Error: input CADU does not exist: " << cadu_path.string() << "\n";
        return 1;
    }

    // Output dir: <cadu_dir>/MSU-MR/quality
    const std::filesystem::path cadu_dir = cadu_path.parent_path();
    const std::filesystem::path msu_dir = cadu_dir / "MSU-MR";

    if (!std::filesystem::exists(msu_dir) || !std::filesystem::is_directory(msu_dir)) {
        std::cerr << "Error: expected folder 'MSU-MR' to exist beside the CADU file.\n" << "  CADU:   " << cadu_path.string() << "\n" << "  Missing:" << msu_dir.string() << "\n";
        return 1;
    }

    const std::filesystem::path quality_dir = msu_dir / "quality";
    std::error_code ec;
    std::filesystem::create_directories(quality_dir, ec);
    if (ec) {
        std::cerr << "Error: failed to create quality directory:\n" << "  " << quality_dir.string() << "\n" << "  " << ec.message() << "\n";
        return 1;
    }

    try {
        MSUMRQFReconstructor recon;
        extract_from_cadu(cadu_path, recon);

        std::vector<std::pair<uint16_t, ChannelStats>> stats_json;
        std::vector<std::pair<uint16_t, int>> present; // (apid, channel)

        // Write per-channel TIFFs (raw QF, missing=0)
        for (uint16_t apid = 64; apid <= 69; apid++) {
            const int ch = apid_to_channel(apid);

            std::vector<uint8_t> pixels14;
            uint32_t w = 0, h = 0;
            ChannelStats st;

            if (!recon.build_qf_tif_for_channel(ch, pixels14, w, h, st)) continue;

            const auto out_path = quality_dir / CH_OUT_NAMES[ch];
            if (!write_tiff_gray8(out_path, w, h, pixels14)) {
                std::cerr << "Error: failed to write TIFF: " << out_path.string() << "\n";
                continue;
            }

            stats_json.push_back({ apid, st });
            present.push_back({ apid, ch });

            if (st.has_values)
                std::cout << "Wrote " << out_path.string() << " : " << w << "x" << h << "  QF min=" << int(st.minv) << " max=" << int(st.maxv) << " avg=" << std::fixed << std::setprecision(3) << st.avg << "\n";
            else
                std::cout << "Wrote " << out_path.string() << " : " << w << "x" << h << " (no nonzero QF values)\n";
        }

        // Write JSON
        {
            const auto json_path = quality_dir / "quality.json";
            if (!write_quality_json(json_path, stats_json)) {
                std::cerr << "Error: failed to write JSON: " << json_path.string() << "\n";
                return 1;
            }
            std::cout << "Wrote " << json_path.string() << "\n";
        }

        // Composite: pick 3 smallest APIDs present
        std::sort(present.begin(), present.end(), [](const auto& a, const auto& b) { return a.first < b.first; });

        if (present.size() < 3) {
            std::cout << "Note: fewer than 3 channels present; skipping RGB composite.\n";
        }
        else {
            if (present.size() > 3) std::cout << "Warning: more than 3 channels present (possible channel switch). " << "Using the three smallest APIDs for RGB.\n";

            const std::array<uint16_t, 3> apids = { present[0].first, present[1].first, present[2].first };
            const std::array<int, 3> chs = { present[0].second, present[1].second, present[2].second };

            std::vector<uint8_t> rgb;
            int cw = 0, ch = 0;
            std::string digits;

            if (!build_rgb_composite_from_segments(recon, chs, apids, rgb, cw, ch, digits)) {
                std::cerr << "Error: failed to build RGB composite buffer.\n";
                return 1;
            }

            const std::string jpg_name = std::string("msu_mr_rgb_") + digits + "_qf.jpg";
            const auto jpg_path = quality_dir / jpg_name;

            constexpr int JPG_QUALITY = 90;
            if (!write_jpeg_rgb_turbo(jpg_path, rgb, cw, ch, JPG_QUALITY)) {
                std::cerr << "Error: failed to write JPEG (libjpeg-turbo): " << jpg_path.string() << "\n";
                return 1;
            }

            std::cout << "Wrote " << jpg_path.string() << " : " << cw << "x" << ch << "  (R,G,B APIDs = " << apids[0] << "," << apids[1] << "," << apids[2] << ")" << "  Q=" << JPG_QUALITY << "\n";
        }

        if (!recon.apids_seen.empty()) {
            std::cout << "Channel APIDs (secondary header present): ";
            bool first = true;
            for (auto apid : recon.apids_seen) {
                if (!first) std::cout << ", ";
                std::cout << apid;
                first = false;
            }
            std::cout << "\n";
        }
        else {
            std::cout << "No MSU-MR channel packets (APIDs 64..69, secondary header present) were found.\n";
        }

        std::cout << "Output dir: " << std::filesystem::absolute(quality_dir).string() << "\n";
        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "Fatal: " << e.what() << "\n";
        return 1;
    }
}
