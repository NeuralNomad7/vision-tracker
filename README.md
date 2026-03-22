# Vision Tracker

[![CI](https://github.com/NeuralNomad7/vision-tracker/actions/workflows/ci.yml/badge.svg)](https://github.com/NeuralNomad7/vision-tracker/actions/workflows/ci.yml)
[![Python 3.11+](https://img.shields.io/badge/python-3.11%2B-blue.svg)](https://www.python.org/downloads/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

Real-time robotic perception system that tracks brightly colored objects via webcam and outputs target vectors for robotic arm control. Built with OpenCV and hardened for production use.

## Features

- **Kalman-filtered tracking** — 4-state filter (position + velocity) for smooth, predictive object tracking
- **Multi-object tracking** — tracks up to 10 objects simultaneously with persistent IDs and unique colors
- **Serial robot output** — streams target vectors to a microcontroller at 30 Hz over a configurable serial port
- **CSV data logging** — millisecond-precision logs with Kalman state, raw measurements, velocity, and distance
- **HSV calibration sliders** — live trackbar UI for tuning color detection to any lighting condition
- **Distance estimation** — pinhole camera model estimates object distance in centimeters
- **Production-hardened** — graceful signal handling, input validation, serial disconnect recovery, throttled output

## Architecture

```
┌──────────┐    ┌──────────────┐    ┌──────────────┐    ┌────────────┐
│  Webcam  │───>│ HSV Masking  │───>│  Contour     │───>│  Kalman    │
│  (0-N)   │    │ (2-channel)  │    │  Detection   │    │  Filter    │
└──────────┘    └──────────────┘    └──────────────┘    └─────┬──────┘
                      ▲                                       │
                      │                                       ▼
               ┌──────┴───────┐                  ┌────────────────────┐
               │ HSV Sliders  │                  │  Multi-Object      │
               │ (live tune)  │                  │  Matcher (greedy   │
               └──────────────┘                  │  nearest-neighbor) │
                                                 └─────┬──────┬──────┘
                                                       │      │
                                          ┌────────────┘      └────────────┐
                                          ▼                                ▼
                                   ┌─────────────┐                 ┌──────────────┐
                                   │ Serial Out   │                 │  CSV Logger   │
                                   │ (30 Hz)      │                 │  (every frame)│
                                   └─────────────┘                 └──────────────┘
```

## Quick Start

### Install

```bash
git clone https://github.com/NeuralNomad7/vision-tracker.git
cd vision-tracker
python -m venv venv
source venv/bin/activate   # Windows: venv\Scripts\activate
pip install -r requirements.txt
```

### Run

```bash
# Basic — webcam only, visual output
python vision_tracker.py

# With CSV logging (auto-timestamped)
python vision_tracker.py --csv-auto

# With serial output to a robot controller
python vision_tracker.py --serial COM3 --baud 115200

# Full stack
python vision_tracker.py --serial /dev/ttyUSB0 --csv-auto --verbose

# List available serial ports
python vision_tracker.py --list-ports
```

### CLI Reference

| Flag | Description | Default |
|------|-------------|---------|
| `--serial PORT` | Serial port for robot output | None (disabled) |
| `--baud RATE` | Serial baud rate | 115200 |
| `--list-ports` | List serial ports and exit | — |
| `--csv FILE` | CSV log output path (must be under cwd) | None |
| `--csv-auto` | Auto-generate timestamped CSV in `./logs/` | Off |
| `--camera N` | Camera index | 0 |
| `--verbose, -v` | DEBUG-level logging | Off |

### Keyboard Shortcuts

| Key | Action |
|-----|--------|
| `q` | Quit |
| `m` | Toggle mask view (see raw detection) |
| `r` | Reset object IDs |
| `s` | List serial ports in terminal |

## Serial Protocol

Target vectors are streamed as ASCII text at up to 30 Hz:

```
T<id>,<vec_x>,<vec_y>,<dist_cm>,<vel_x>,<vel_y>\n
F\n
```

| Field | Type | Description |
|-------|------|-------------|
| `id` | int | Object ID (1–9999) |
| `vec_x` | signed int | Pixels right (+) or left (−) of frame center |
| `vec_y` | signed int | Pixels up (+) or down (−) of frame center |
| `dist_cm` | float | Estimated distance in cm |
| `vel_x` | float | Kalman velocity X (px/frame) |
| `vel_y` | float | Kalman velocity Y (px/frame) |

`F\n` marks end-of-frame. Example stream:

```
T1,+42,-18,65.3,+1.2,-0.5
T2,-130,+90,102.1,+0.0,+0.3
F
```

## CSV Schema

| Column | Type | Description |
|--------|------|-------------|
| `timestamp` | ISO 8601 | Millisecond-precision local time |
| `epoch_ms` | int | Unix epoch in milliseconds |
| `obj_id` | int | Tracked object ID |
| `kalman_x` | int | Kalman-filtered X pixel position |
| `kalman_y` | int | Kalman-filtered Y pixel position |
| `raw_x` | int | Raw measured X pixel position |
| `raw_y` | int | Raw measured Y pixel position |
| `vec_x` | int | Target vector X (from frame center) |
| `vec_y` | int | Target vector Y (from frame center, Y-up) |
| `vel_x` | float | Kalman velocity X |
| `vel_y` | float | Kalman velocity Y |
| `dist_cm` | float | Estimated distance (cm) |
| `bbox_w` | int | Bounding box width (px) |
| `bbox_h` | int | Bounding box height (px) |

## Calibration

### Distance Estimation

The tracker uses a pinhole camera model. To calibrate for your setup:

1. Hold a known object (e.g., 7.6 cm sticky note) at a known distance (e.g., 50 cm)
2. Note the object's pixel width in the bounding box
3. Calculate: `focal_length = (pixel_width * distance_cm) / real_width_cm`
4. Update `FOCAL_LENGTH_PX` and `KNOWN_WIDTH_CM` at the top of `vision_tracker.py`

### HSV Color Tuning

1. Run the tracker — the **HSV Calibration** window opens alongside the feed
2. Press `m` to toggle the binary mask view
3. Adjust the G (green) and P (pink) channel sliders until only your target object is white in the mask
4. Note the final slider values and update `HSV_DEFAULTS` in the source for persistence

## Project Structure

```
vision-tracker/
├── .github/workflows/ci.yml   # GitHub Actions CI pipeline
├── logs/                       # CSV tracking logs (gitignored)
├── vision_tracker.py           # Main application
├── requirements.txt            # Pinned Python dependencies
├── setup.cfg                   # Flake8 linting configuration
├── LICENSE                     # MIT License
└── README.md
```

## Contributing

1. Fork the repo
2. Create a feature branch (`git checkout -b feature/my-feature`)
3. Make changes and ensure `flake8 vision_tracker.py` passes
4. Commit and push
5. Open a Pull Request

## License

[MIT](LICENSE)
