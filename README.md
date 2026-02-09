# lrpt_qf_extract

A small utility that extracts JPEG quality factors (QFs) from METEOR M2-x LRPT CADU files.

## Usage
```powershell
lrpt_qf_extract.exe "path\to\file.cadu"
```

## What it does
Given a `.cadu` file, the tool:
- Reconstructs MSU-MR segment ordering using SatDump-equivalent logic (APIDs 64–69)
- Writes per-channel QF maps as **8-bit grayscale TIFFs** (14×N; 1 pixel per segment)
- Writes `quality.json` (min/max/avg per channel, ignoring missing/zero)
- Writes a **QF heatmap JPG** (1568×H) by averaging QF across up to the three smallest APIDs present
  - Heat color ramp: red → yellow → green (QF clamped to 20..100); missing segments are black (0)
  - Heatmap filename encodes channels: `msu_mr_qf_heatmap_###.jpg` where APID 64→1 … 69→6

## Output layout
The tool expects an `MSU-MR` folder **beside** the input CADU file. It will create `MSU-MR\quality` and write outputs there:

- `MSU-MR-1_qf.tif` … `MSU-MR-6_qf.tif` (only for channels present)
- `quality.json`
- `msu_mr_qf_heatmap_###.jpg`

## Scaling behavior
- TIFFs store **raw QF** values (typically ~20–100). Missing segments are **0**.
- The heatmap clamps QF to **20..100** and maps it to a red→yellow→green ramp; missing segments (**0**) render as black.
  - If multiple channels are used for the heatmap, per-segment QF is the **average of nonzero** QFs across those channels.

## Build
Requires libjpeg-turbo (TurboJPEG). Build with any C++17 compiler; link against turbojpeg.
