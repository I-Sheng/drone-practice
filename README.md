# Drone Practice — DJI Tello Autonomous Gate Passing

A ROS-based project for autonomous DJI Tello drone navigation through visual gates using computer vision and control logic.

---

## Technique & Platform

### Hardware
- **DJI Tello** — a lightweight, Wi-Fi-connected drone with a built-in 720p camera and onboard H.264 video encoder. The Tello communicates over UDP; all commands and telemetry are exchanged via the Tello ROS driver.

### Software & Middleware
| Layer | Technology | Role |
|---|---|---|
| Middleware | **ROS (Robot Operating System)** | Topic-based communication between nodes |
| Vision | **OpenCV** | HSV color filtering, contour detection, bounding rect |
| Video decode | **PyAV (libav)** | Decodes the H.264 stream from the Tello camera |
| Drone control | **Tello ROS Driver** | Exposes `/tello/cmd_vel`, `/tello/status`, video topics |
| FSM | **python-statemachine** | Declarative finite state machine for gate passing |
| Numerics | **NumPy** | Array operations on image frames |

### Core Techniques
- **HSV Color Filtering** — Red is detected by masking two hue ranges (0–5° and 175–180°) to handle the HSV hue wraparound at red.
- **Contour Detection & minAreaRect** — `cv2.findContours` extracts the gate outline; `cv2.minAreaRect` gives its center and bounding area.
- **Area-ratio Threshold** — Gate area / total frame area ≥ 0.35 triggers the `canPass` flag, indicating the drone is close enough to fly through.
- **Alignment Control Loop** — Computes pixel offsets (dx, dy) from image center (480, 200) and corrects left/right (`linear.x`) and up/down (`linear.z`) until within threshold.
- **Finite State Machine (FSM)** — Four states (`hover → correction → forward → addSp`) make the control logic explicit and easier to extend.

---

## Project Structure

```
drone-practice/
├── README.md
├── simple_demo.py          # Standalone flight demo using tellopy (no ROS required)
├── basic/                  # Baseline: simple flight + vision display only
│   ├── simple_tello.py     # Tello wrapper (state tracking, publishers for takeoff/land/flip/move)
│   ├── test_h264_sub.py    # Vision node — detects gate, displays result, no publishing
│   └── run_tello.py        # Simple flight sequence (takeoff → flip → land)
└── gate_pass/              # Full pipeline: vision + autonomous gate passing
    ├── simple_tello.py     # Tello wrapper + /target_point subscriber
    ├── test_h264_sub.py    # Vision node — detects gate, publishes /target_point
    ├── pass_example.py     # Control-loop gate passing
    └── fsm_pass_example.py # FSM-based gate passing
```

---

## ROS Data Flow

```
/tello/image_raw/h264  (CompressedImage)
        │
        ▼
  test_h264_sub.py          ← HSV filter → contour → center offset → canPass
        │
        ▼
  /target_point             ← Float64MultiArray: [center_x, center_y, canPass]
        │
        ▼
  pass_example.py           ← alignment correction + forward motion
  (or fsm_pass_example.py)
        │
        ▼
  /tello/cmd_vel            ← Twist commands
        │
        ▼
    Tello Drone
```

---

## Prerequisites

| Requirement | Notes |
|---|---|
| ROS Melodic / Noetic | Standard installation |
| Python 3.6+ | (or 2.7 for `basic/` nodes) |
| DJI Tello | Connected via Wi-Fi |
| Tello ROS driver | Provides `/tello/image_raw/h264` and `/tello/cmd_vel` |

**Python dependencies:**

```bash
pip install opencv-python av numpy python-statemachine tellopy
```

---

## Running the Project

### Gate Passing (full pipeline)

```bash
# Terminal 1 — ROS core
roscore

# Terminal 2 — Vision node
python3 gate_pass/test_h264_sub.py

# Terminal 3 — Control node (choose one)
python3 gate_pass/pass_example.py       # control-loop approach
python3 gate_pass/fsm_pass_example.py   # FSM approach
```

### Basic Flight Demo

```bash
# Terminal 1 — ROS core
roscore

# Terminal 2 — Simple takeoff / flip / land
python3 basic/run_tello.py
```

### Standalone Demo (no ROS)

```bash
python3 simple_demo.py
```

---

## How It Works

### 1. Gate Detection (`test_h264_sub.py`)

- Decodes the H.264 video stream frame-by-frame using **PyAV**
- Converts each frame to **HSV** and masks the red hue range (accounts for wraparound at 0°/180°)
- Finds the largest red contour with `cv2.findContours` + `cv2.minAreaRect`
- Computes the center offset from image center `(480, 200)` and gate area ratio
- Publishes `canPass = 1` when gate area ≥ 35% of total frame area

### 2. Control Loop (`pass_example.py`)

| Phase | Condition | Action |
|---|---|---|
| Wait | `target == -1` | Hold until first frame arrives |
| Correction | `\|dx\| ≥ 24 or \|dy\| ≥ 24` | Correct left/right/up/down at 0.1–0.2 m/s |
| Forward | aligned | Move forward at 0.3 m/s |
| Boost | `canPass == 1` | Accelerate at 0.4 m/s for 5.2 s, then stop |

### 3. FSM Controller (`fsm_pass_example.py`)

| State | Behavior | Transition |
|---|---|---|
| `hover` | Hold position | Gate detected → `correction` or `forward` |
| `correction` | Align to gate center | Aligned → `forward`; canPass → `addSp` |
| `forward` | Fly toward gate | De-aligned → `correction`; canPass → `addSp` |
| `addSp` | Speed boost through gate | Done → exit |

---

## Key Parameters

| Parameter | Value | File |
|---|---|---|
| Image center | `(480, 200)` | `gate_pass/pass_example.py` |
| Alignment threshold (narrow) | `24 px` | `gate_pass/pass_example.py` |
| Alignment threshold (wide x) | `60 px` | `gate_pass/pass_example.py` |
| Alignment threshold (wide y) | `30 px` | `gate_pass/pass_example.py` |
| canPass area ratio | `0.35` (35%) | `gate_pass/test_h264_sub.py` |
| Forward speed | `0.3 m/s` | `gate_pass/pass_example.py` |
| Boost speed | `0.4 m/s` | `gate_pass/pass_example.py` |
| Startup frame skip | `300 frames` | `gate_pass/test_h264_sub.py` |

---

## ROS Topics

| Topic | Type | Direction |
|---|---|---|
| `/tello/image_raw/h264` | `CompressedImage` | Subscribed (vision node) |
| `/target_point` | `Float64MultiArray` | Published by vision / subscribed by control |
| `/tello/cmd_vel` | `Twist` | Published (control node) |
| `/tello/takeoff` | `Empty` | Published |
| `/tello/land` | `Empty` | Published |
| `/tello/emergency` | `Empty` | Published |
| `/tello/flip` | `UInt8` | Published |
| `/tello/status` | `TelloStatus` | Subscribed (state tracking) |
