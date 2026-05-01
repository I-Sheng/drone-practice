# Drone Practice — DJI Tello Autonomous Gate Passing

A ROS-based project for autonomous DJI Tello drone navigation through visual gates using HSV color detection and a Finite State Machine controller.

---

## Overview

The drone detects a **red-colored gate/frame** in its camera feed, aligns to its center, and autonomously flies through it. Two control approaches are implemented:

- **Control loop** (`pass_example.py`) — continuous PID-style alignment correction
- **Finite State Machine** (`fsm_pass_example.py`) — explicit state transitions: `hover → correction → forward → addSp`

---

## Architecture

```
/tello/image_raw/h264  (CompressedImage)
        │
        ▼
  test_h264_sub.py          ← OpenCV HSV detection, computes gate center
        │
        ▼
  /target_point             ← Float64MultiArray: [center_x, center_y, canPass]
        │
        ▼
  pass_example.py           ← drone control logic
  (or fsm_pass_example.py)
        │
        ▼
  /tello/cmd_vel            ← Twist commands
        │
        ▼
    Tello Drone
```

---

## Project Structure

```
drone-practice/
├── code-pass/                 # Full pipeline: vision + gate passing
│   ├── test_h264_sub.py       # Vision node — detects gate, publishes target_point
│   ├── pass_example.py        # Control node — aligns and flies through gate
│   └── simple_tello.py        # Tello wrapper with ROS topic integration
│
├── code-no-pass/              # Baseline: vision display only, no gate passing
│   ├── test_h264_sub.py       # Vision node — detects gate, display/record only
│   ├── week7-run_tello.py     # Simple flight demo (takeoff, flip, land)
│   └── simple_tello.py        # Tello wrapper (state tracking only)
│
├── fsm_pass_example.py        # FSM-based gate-passing controller
└── simple_demo.py             # Standalone demo using tellopy (no ROS)
```

---

## Prerequisites

| Requirement | Notes |
|---|---|
| ROS Melodic / Noetic | Standard installation |
| Python 3.6+ | (or 2.7 for legacy nodes) |
| DJI Tello | Connected via Wi-Fi |
| Tello ROS driver | Provides `/tello/image_raw/h264` and `/tello/cmd_vel` |

**Python dependencies:**

```bash
pip install opencv-python av numpy python-statemachine tellopy
```

---

## Running the Project

### Gate Passing (vision + control)

Open three terminals:

```bash
# Terminal 1 — ROS core
roscore

# Terminal 2 — Vision node (detects gate, publishes /target_point)
python3 code-pass/test_h264_sub.py

# Terminal 3 — Control node (aligns drone, flies through gate)
python3 code-pass/pass_example.py
```

To use the FSM controller instead:

```bash
python3 fsm_pass_example.py
```

### Simple Flight Demo (no ROS required)

```bash
python3 simple_demo.py
```

---

## How It Works

### 1. Gate Detection (`test_h264_sub.py`)

- Decodes the H.264 video stream using **PyAV**
- Converts each frame to **HSV color space**
- Masks the red channel (accounts for HSV hue wraparound at 0°/180°)
- Finds the largest red contour via `minAreaRect`
- Computes the **center offset** from image center `(480, 200)`
- Publishes `canPass = 1` when the gate occupies >35% of the frame area

### 2. Control Loop (`pass_example.py`)

- Reads `[center_x, center_y, canPass]` from `/target_point`
- **Narrow threshold (24 px):** corrects left/right/up/down
- **Wide threshold (60 px x, 30 px y):** maintains alignment during forward motion
- When aligned and `canPass == 1`, boosts speed to fly through the gate

### 3. FSM Controller (`fsm_pass_example.py`)

| State | Behavior | Transition |
|---|---|---|
| `hover` | Hold position | Gate detected |
| `correction` | Align to gate center | Within threshold |
| `forward` | Fly toward gate | canPass flag set |
| `addSp` | Speed boost through gate | Gate cleared |

---

## Key Parameters

| Parameter | Value | Location |
|---|---|---|
| Image center | `(480, 200)` | `pass_example.py` |
| Alignment threshold (narrow) | `24 px` | `pass_example.py` |
| Alignment threshold (wide x) | `60 px` | `pass_example.py` |
| Alignment threshold (wide y) | `30 px` | `pass_example.py` |
| canPass area ratio | `0.35` (35%) | `test_h264_sub.py` |
| Forward speed | `0.1–0.3 m/s` | `pass_example.py` |
| Boost speed | `0.5 m/s` | `pass_example.py` |

---

## ROS Topics

| Topic | Type | Direction |
|---|---|---|
| `/tello/image_raw/h264` | `CompressedImage` | Subscribed (vision node) |
| `/target_point` | `Float64MultiArray` | Published (vision) / Subscribed (control) |
| `/tello/cmd_vel` | `Twist` | Published (control node) |
| `/tello/takeoff` | `Empty` | Published |
| `/tello/land` | `Empty` | Published |
| `/tello/emergency` | `Empty` | Published |
| `/tello/flip` | `UInt8` | Published |
