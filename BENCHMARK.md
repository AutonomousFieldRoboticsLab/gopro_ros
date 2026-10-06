# gopro_to_rosbag conversion speed

Benchmark of the current converter and the expected effect of the proposed speed-ups.

## Setup

| | |
|---|---|
| CPU | Intel Core Ultra 7 155H (22 threads) |
| RAM | 62 GB |
| GPU | NVIDIA RTX 500 Ada Laptop (4 GB) |
| OS / ROS | Ubuntu 24.04, ROS 2 Jazzy |
| FFmpeg | 6.1 (libavcodec 60.31) |
| Videos | HEVC 1920x1080, 29.97 fps |
| Converter settings | `scale:=0.5 grayscale:=true compressed_image_format:=true`, MCAP (`zstd_fast`) |

One conversion at a time, nothing else running.

## Current converter (measured)

| Video | Length | Bitrate | Frames | Time | Throughput | Speed vs real time |
|---|---|---|---|---|---|---|
| `GX010005` | 11.9 min | 45 Mbps | 21,330 | **201 s** | 106 fps | 3.5x |
| `GX020005` | 2.6 min | 45 Mbps | 4,737 | **43 s** | 109 fps | 3.6x |
| `Mult` (both chapters above) | 14.5 min | 45 Mbps | 26,067 | **246 s** | 106 fps | 3.5x |
| `GX010285` (ROS 1 Noetic, color) | 50.2 min | 0.9 Mbps | 90,240 | **415 s** | 217 fps | 7.3x |

The output format barely matters. On `GX020005` all of these took 43–44 s:

| Images | Storage | Time | Size |
|---|---|---|---|
| JPEG | MCAP + zstd_fast | 43.3 s | 632 MB |
| JPEG | MCAP, no compression | 43.4 s | 641 MB |
| JPEG | SQLite3 (db3) | 44.3 s | 646 MB |
| Raw | MCAP, no compression | 43.2 s | 2.3 GB |

### Where the time goes (`GX010005`, 201 s)

| Stage | Time | Share | Threads |
|---|---|---|---|
| `sws_scale` (resize + YUV to RGB) | 86 s | 43% | 1 |
| JPEG encode + bag write | 54 s | 27% | 1 |
| Waiting on the decoder | 42 s | 21% | decoder threads |
| `cvtColor` (RGB to BGR to GRAY) | 15 s | 7% | 1 |
| GPMF/IMU parsing + IMU write | 0.5 s | <1% | 1 |

All per-frame work after decoding runs one frame at a time on a single thread. For grayscale, every
frame is converted YUV to RGB to BGR to GRAY, even though the luma (Y) plane of the video already is
the gray image.

## Decoding limit (measured)

FFmpeg decoding only, first 120 s of each video, frames copied back to the CPU.

| Video | Bitrate | CPU decode | NVDEC (NVIDIA) decode |
|---|---|---|---|
| `GX010285` | 0.9 Mbps | 954 fps | 991 fps |
| `GX010079` | 3.9 Mbps | 464 fps | 1,039 fps |
| `GX010005` | 45 Mbps | 115 fps | 606 fps |

Intel VAAPI (Arc iGPU) reached about 450 fps on 45 Mbps video.

Decoding cost depends strongly on bitrate. At 45 Mbps the CPU decoder alone already needs about
185 s for `GX010005`, so the current converter (201 s) is close to the decoding limit there. At low
and medium bitrate the decoder is fast and the single-threaded per-frame work is the bottleneck.

## Proposed speed-ups

1. **Cheaper conversion:** for grayscale, resize the Y plane directly; for color, convert straight to
   BGR (no extra `cvtColor`), multi-threaded.
2. **Parallel JPEG encoding:** a pool of worker threads, with a single writer keeping frames in order.
3. **Hardware decoding (NVDEC):** optional, with an automatic fallback to CPU decoding. Docker
   services get `gpus: all` (the NVIDIA container runtime is installed).

## Expected times (estimated)

Estimated from the decoding limits above, assuming the optimized converter runs at 85-100% of the
decoder's speed (frames / decode fps, then allowing about 15% overhead).

| Video | Length | Bitrate | Now | Steps 1-2 (CPU) | Steps 1-3 (+ NVDEC) |
|---|---|---|---|---|---|
| `GX010285` | 50.2 min | 0.9 Mbps | 6.9 min (measured) | ~2 min | ~1.8 min |
| `GX010079` | 50.2 min | 3.9 Mbps | ~10-14 min | ~4 min | ~1.7 min |
| `GX010005` | 11.9 min | 45 Mbps | 3.4 min (measured) | ~3.1 min | ~0.8 min |
| `Mult` | 14.5 min | 45 Mbps | 4.1 min (measured) | ~3.8 min | ~1 min |

- **Low and medium bitrate:** steps 1-2 alone give about 3x; NVDEC adds a little on top.
- **High bitrate:** CPU decoding is the limit, so only NVDEC helps, at about 4x.
