# Two Towers - Autonomous Tracking System

> **Status**: Single-tower tracking operational. Multi-tower coordination architecture planned.

A high-performance autonomous person tracking system featuring YOLOv8 computer vision, behavior tree control architecture, and real-time servo actuation. Built for Raspberry Pi 4 with hardware PWM control.

[![CI](https://github.com/tatelloyd/two-towers/actions/workflows/ci.yml/badge.svg)](https://github.com/tatelloyd/two-towers/actions/workflows/ci.yml)
[![Platform](https://img.shields.io/badge/platform-Raspberry%20Pi%204-red)]()
[![OS](https://img.shields.io/badge/Ubuntu-24.04%20LTS%20Server-orange)]()
[![ROS2](https://img.shields.io/badge/ROS%202-Jazzy-green)]()
[![C++](https://img.shields.io/badge/C%2B%2B-17-blue)]()
[![Python](https://img.shields.io/badge/Python-3.12-blue)]()

---

## Project Overview

Two Towers demonstrates core principles of autonomous systems engineering:

- **Behavior Tree Architecture**: Modular decision-making framework enabling reactive autonomy
- **Real-Time Vision Pipeline**: YOLOv8 object detection feeding ROS 2 message passing — measured 8.1 Hz on a Pi 4 (see [Measured Performance](#measured-performance))
- **Hardware PWM Control**: Jitter-free servo actuation via pigpio daemon (50 Hz control loop)
- **Embedded Linux Deployment**: Headless operation on resource-constrained hardware
- **Scalable Design**: Architecture designed for multi-agent extension

### Current Capabilities

- Autonomous person tracking with proportional control
- Real-time YOLOv8n detection optimized for Raspberry Pi
- Smooth servo control with adaptive gains and deadband
- Scanning behavior when target is lost
- Flask-based video streaming for remote monitoring
- ROS2-based communication between detector and tracker

### Planned Features

- Multi-turret coordination with seamless handoff
- Cooperative tracking from multiple viewpoints
- Predictive tracking with Kalman filtering
- AI-powered decision making via Claude API

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    VISION PIPELINE (Python)                 │
│  ┌──────────┐      ┌──────────┐      ┌────────────────┐    │
│  │  USB Cam │─────▶│ YOLOv8n  │─────▶│  ROS2 Topic    │    │
│  │  320x240 │      │ ~8 Hz    │      │  /detections   │    │
│  └──────────┘      └──────────┘      └────────┬───────┘    │
│                                               │            │
│  Optional: Flask video stream on port 5000    │            │
└───────────────────────────────────────────────┼────────────┘
                                                │
                                                ▼
┌───────────────────────────────────────────────┼────────────┐
│              CONTROL SYSTEM (C++)             │            │
│  ┌────────────────────────────────────────────▼───────┐   │
│  │           Tracker Node (20 Hz)                     │   │
│  │  ┌─────────────────────────────────────────────┐   │   │
│  │  │  State Machine: TRACKING <──> SCANNING      │   │   │
│  │  │                                             │   │   │
│  │  │  Proportional Control:                      │   │   │
│  │  │    error = target_pos - center              │   │   │
│  │  │    adjustment = clamp(error * gain, max)    │   │   │
│  │  └─────────────────────────────────────────────┘   │   │
│  └────────────────────┬───────────────────────────────┘   │
│                       ▼                                    │
│  ┌─────────────────────────────────────────────────────┐  │
│  │              Turret Hardware Layer                   │  │
│  │  ┌──────────────────┐    ┌──────────────────┐       │  │
│  │  │ ServoController  │    │ ServoController  │       │  │
│  │  │   (Pan/GPIO 17)  │    │  (Tilt/GPIO 27)  │       │  │
│  │  └────────┬─────────┘    └─────────┬────────┘       │  │
│  └───────────┼────────────────────────┼────────────────┘  │
│              ▼                        ▼                    │
│       ┌──────────┐              ┌──────────┐              │
│       │ Pan Servo│              │Tilt Servo│              │
│       │  SG90    │              │  SG90    │              │
│       └──────────┘              └──────────┘              │
└─────────────────────────────────────────────────────────────┘
```

---

## Hardware Requirements

| Component | Specifications | Notes |
|-----------|---------------|-------|
| **Raspberry Pi 4** | 4GB+ RAM | Main controller |
| **Pan/Tilt Mount** | [SparkFun ROB-14045](https://www.sparkfun.com/products/14045) | Includes 2x SG90 servos |
| **USB Webcam** | 320x240 @ 15fps | Any UVC-compatible camera |
| **Power Supply** | 5V 3A USB-C | Official RPi PSU recommended |
| **MicroSD Card** | 32GB+ Class 10 | Ubuntu 24.04 LTS Server, 64-bit (arm64) |

> **Flash the card with Raspberry Pi Imager**, choosing *Ubuntu Server 24.04 LTS
> (64-bit)* — not Desktop, and not Raspberry Pi OS. Use the gear icon to preset
> the username, password and SSH before writing; the alternative is Ubuntu's
> first-boot forced password change, which disconnects you mid-login.
>
> ROS 2 Jazzy targets Ubuntu 24.04 specifically. On any other release there are
> no `ros-jazzy-*` packages to install.

---

## Quick Start

### Installation

On a Pi freshly flashed with Ubuntu 24.04 Server, one script does everything:

```bash
git clone https://github.com/tatelloyd/two-towers.git
cd two-towers
./scripts/bootstrap_pi.sh
```

That adds the ROS 2 apt repository, installs Jazzy and BehaviorTree.CPP,
**builds pigpio from source** and enables the daemon (Ubuntu has no working
pigpio package — see Troubleshooting), builds the virtualenv, installs the
Python dependencies, verifies that `rclpy`, `cv2`, `flask` and `ultralytics`
all import, and builds the workspace. It is idempotent — re-running it is
safe, which is what makes it testable on a second card.

On a laptop with no Pi attached, stub the GPIO layer instead:

```bash
./scripts/bootstrap_pi.sh --sim-gpio
```

Servo commands become no-ops; everything above the GPIO boundary is real. This
is also exactly what CI builds.

<details>
<summary>What the script does, if you would rather do it by hand</summary>

```bash
# ROS 2 Jazzy apt repository. Note this uses the ros2-apt-source package --
# the older ros-archive-keyring.gpg method stopped working when that key
# expired in mid-2025, and most tutorials online still show it.
sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
export ROS_APT_SOURCE_VERSION=1.2.0
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.noble_all.deb"
sudo apt install -y /tmp/ros2-apt-source.deb

# libgl1 and libglib2.0-0t64 are for opencv-python, which links OpenGL that a
# Server image does not ship. Note the ABSENCE of pigpio here -- see below.
sudo apt update && sudo apt install -y \
    ros-jazzy-ros-base ros-jazzy-behaviortree-cpp \
    python3-colcon-common-extensions python3-venv \
    build-essential cmake git libgl1 libglib2.0-0t64

# pigpio must be built from source: Ubuntu 24.04 has no pigpiod package, and
# its libpigpiod-if-dev ships a header that includes a pigpio.h no noble
# package provides. `apt install pigpio` is Raspberry Pi OS advice.
git clone --depth 1 --branch v79 https://github.com/joan2937/pigpio.git /tmp/pigpio
(cd /tmp/pigpio && make -j"$(nproc)" && sudo make install && sudo ldconfig)

# Hardware PWM needs the daemon. Turret's constructor throws without it.
# The source build ships no unit file, so install the one in this repo.
sudo install -m0644 deploy/systemd/pigpiod.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now pigpiod

# --system-site-packages is MANDATORY. rclpy is installed by apt into
# /opt/ros/jazzy and has no PyPI package, so an isolated venv cannot see it
# and the detector dies on `import rclpy` before it ever loads a model.
python3 -m venv --system-site-packages venv
./venv/bin/pip install -r requirements.txt

source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

</details>

BehaviorTree.CPP is a hard requirement — the tracker is a behavior tree node
and there is no fallback. If it is missing, the build fails at `find_package`
rather than quietly producing a package with no tracker in it. The version
floor is 4.1, because the tracker calls `Tree::tickOnce()` and its XML declares
`BTCPP_format="4"`. The Jazzy apt repository currently carries 4.9.

### Running the System

One tower, one command:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source venv/bin/activate
ros2 launch two_towers tower.launch.py
```

This brings up the perception node and the behavior tree tracker together in
the `/tower_a` namespace. To run tower B instead:

```bash
ros2 launch two_towers tower.launch.py tower_id:=tower_b
```

Both towers at once (see the note in `launch/two_towers.launch.py` about
inference rate on a single Pi):

```bash
ros2 launch two_towers two_towers.launch.py
```

Per-tower settings -- GPIO pins, camera FOV, stream port -- live in
`config/tower_a.yaml` and `config/tower_b.yaml`. Neither node contains a
tower-specific string; identity comes entirely from the launch namespace.

The turret will:
1. Scan back and forth searching for a person
2. Lock onto detected person and track
3. Apply proportional control to keep target centered
4. Resume scanning if target is lost

**Video Stream**: Open `http://<raspberry-pi-ip>:5000` in a browser

**Stop**: Press `Ctrl+C` in either terminal (servos auto-center on shutdown)

### Running as a Service

The commands above keep the robot alive only as long as the SSH session is
open. To have a tower start at boot and stay up:

```bash
sudo ./scripts/install_systemd.sh
sudo systemctl enable --now two-towers@tower_a
journalctl -u two-towers@tower_a -f
```

The unit is templated on the tower id, so `two-towers@tower_b` is a second
instance of the same file rather than a second file. Shared configuration lives
in `/etc/two-towers/tower.env`; per-tower overrides go in
`/etc/two-towers/<tower_id>.env`, which only needs to contain the keys that
differ.

`install_systemd.sh` will not overwrite an existing `tower.env` — once it
exists it belongs to the deployment. Use `--dry-run` to see exactly what would
be written.

If a tower fails under systemd, run the same thing by hand to see the error
directly instead of through the journal:

```bash
TOWER_ID=tower_a TWO_TOWERS_WORKSPACE=$PWD TWO_TOWERS_VENV=$PWD/venv \
  ./scripts/run_tower.sh
```

---

## Control Algorithm

### Proportional Tracking

The tracker uses adaptive proportional control with error-dependent gains:

```cpp
// Error from frame center (normalized 0-1 coordinates)
double x_error = -(target_x - 0.5);  // Inverted for camera orientation
double y_error = -(target_y - 0.5);

// Adaptive gain selection
double gain = |error| > 0.15 ? 8.0 :    // Large error: fast response
              |error| > 0.08 ? 4.0 :    // Medium error: moderate
                               2.0;      // Small error: fine tuning

// Apply with deadband and saturation
if (|error| < 0.05) error = 0;          // 5% deadband
double adjustment = clamp(error * gain, -2.0, 2.0);  // Max 2° per tick
```

### State Machine

```
┌─────────────┐         Target Found         ┌─────────────┐
│  SCANNING   │ ───────────────────────────▶ │  TRACKING   │
│             │                              │             │
│  Sweep pan  │ ◀─────────────────────────── │  Center on  │
│  left/right │      Target Lost (60 frames) │  target     │
└─────────────┘                              └─────────────┘
```

---

## Project Structure

```
two-towers/
├── src/
│   ├── cpp/
│   │   ├── orthanc_tracker_node_bt.cpp  # ROS2 tracker (behavior tree) ← the real one
│   │   ├── orthanc_tracker_node.cpp     # ROS2 tracker (plain proportional)
│   │   ├── Turret.{cpp,hpp}             # Pan/tilt turret controller
│   │   ├── ServoController.{cpp,hpp}    # PWM servo abstraction
│   │   ├── SignalGenerator.{cpp,hpp}    # Sweep waveforms, bringup tool only
│   │   ├── sim/pigpiod_if2.h            # No-op pigpio stub (TWO_TOWERS_SIM_GPIO)
│   │   └── trees/basic_track.xml        # Reference tree (node runs an inline copy)
│   └── python/
│       └── two_towers_detector_node.py  # YOLO detection + Flask
├── msg/
│   ├── Detection.msg                 # Single detection
│   ├── DetectionArray.msg            # Array of detections
│   └── TowerStatus.msg               # Tower state (for multi-agent coordination)
├── launch/
│   ├── tower.launch.py               # One tower, into its own namespace
│   └── two_towers.launch.py          # Both towers
├── config/
│   ├── tower_a.yaml                  # Tower A pins, camera calibration
│   └── tower_b.yaml                  # Tower B pins, camera calibration
├── scripts/
│   ├── bootstrap_pi.sh               # Provision a fresh Pi end to end
│   ├── install_systemd.sh            # Install the unit + /etc/two-towers
│   └── run_tower.sh                  # What the unit executes; runnable by hand
├── deploy/systemd/
│   ├── two-towers@.service           # Templated per-tower unit
│   └── tower.env.template            # Seed for /etc/two-towers/tower.env
├── tests/
│   ├── cpp/menu.cpp                  # Interactive servo bringup tool
│   └── python/test_*.py              # Vision pipeline tests
├── CMakeLists.txt                    # ROS2 build configuration
├── package.xml                       # ROS2 package manifest
└── requirements.txt                  # Python dependencies (pip only; see file)
```

---

## Configuration

Settings come from three places, in increasing order of how much you should
have to rebuild to change them.

### Per-tower parameters (`config/tower_a.yaml`, `config/tower_b.yaml`)

Runtime ROS 2 parameters — no rebuild needed. GPIO pins, camera FOV and stream
port live here, which is why neither node contains a tower-specific string:

```yaml
/**/two_towers_detector:
  ros__parameters:
    enable_streaming: true
    stream_port: 5000
    camera_fov_horizontal_deg: 62.2   # measured, not from a datasheet
    camera_fov_vertical_deg: 48.8

/**/orthanc_tracker_bt:
  ros__parameters:
    pan_pin: 17     # BCM numbering, not physical header position
    tilt_pin: 27
```

Tower B defaults to pins 22/23 and port 5001 so both turrets can be driven from
one Pi's header during bringup.

### Deployment settings (`/etc/two-towers/tower.env`)

Workspace and venv paths, `ROS_DOMAIN_ID`, and extra launch arguments. Read by
the systemd unit; see `deploy/systemd/tower.env.template`.

### Compiled-in constants (require a rebuild)

Control law, in `src/cpp/orthanc_tracker_node_bt.cpp`:

```cpp
const double deadband = 0.08;       // center tolerance, widened to stop oscillation
double max_adj = 1.5 + speed * 0.8; // degrees per tick, capped at 3.0
state.get_predicted_target(0.15, ...);  // 150 ms lookahead for system latency
```

Detection, in `src/python/two_towers_detector_node.py`:

```python
DETECTION_RATE_HZ = 15.0          # Timer setpoint, NOT the achieved rate
YOLO_CONFIDENCE_THRESHOLD = 0.25  # Minimum detection confidence
YOLO_INPUT_SIZE = 160             # Model input (smaller = faster)
```

PWM range is 500–2500 μs for 0°–180°, set in `ServoController`'s constructor
defaults.

---

## Measured Performance

Numbers from a Raspberry Pi 4 (Cortex-A72, 4 GB) running Ubuntu 24.04 Server,
YOLOv8n at 160 px input, 320x240 capture. Measured from the detector's own
periodic log, which reports achieved rate alongside the setpoint:

| Configuration | ms/frame | Achieved |
|---|---|---|
| `enable_streaming:=true` (default) | 167.5 | **6.0 Hz** |
| `enable_streaming:=false` | 123.3 | **8.1 Hz** |

**`DETECTION_RATE_HZ = 15.0` is a timer setpoint, not an achieved rate.** YOLO
inference alone costs ~120 ms on this hardware against a 66.7 ms timer period,
so the timer overruns every tick and the loop free-runs at whatever inference
allows. The 15 Hz figure this README used to advertise was never measured on
the robot; it is retained as a target because it is what NCNN export should
make reachable.

The MJPEG debug stream costs **44 ms/frame — 26% of the loop.** Annotation
draws every box, the crosshair and both zone rectangles inside the detection
callback, then JPEG-encodes at quality 85, all competing with inference. Run
with `enable_streaming:=false` for tracking quality; turn it on to debug what
the detector sees.

Reaching 15 Hz needs a 1.85x speedup from the streaming-off baseline.
Ultralytics' NCNN export is the intended route and typically gives 2-3x on
this hardware; it is not yet implemented here.

---

## Troubleshooting

### Servo not moving
```bash
# Check pigpiod is running
sudo systemctl status pigpiod

# Restart if needed
sudo systemctl restart pigpiod
```

### Camera not detected
```bash
# List video devices
ls /dev/video*

# Test camera
python3 -c "import cv2; print(cv2.VideoCapture(0).isOpened())"
```

### ROS2 topics not connecting
```bash
# List active topics
ros2 topic list

# Echo detection messages
ros2 topic echo /tower_a/detections
```

If two towers on separate Pis cannot see each other's topics, check
`ROS_DOMAIN_ID` matches on both (`/etc/two-towers/tower.env`, default 42). If
they match and it still fails over Wi-Fi but works over Ethernet, the cause is
usually the access point dropping or rate-limiting multicast between wireless
clients — DDS discovery depends on it.

### `ModuleNotFoundError: No module named 'rclpy'`

The venv was created without `--system-site-packages`. rclpy is installed by
apt into `/opt/ros/jazzy` and has no PyPI package, so an isolated venv cannot
see it. Re-running `./scripts/bootstrap_pi.sh` detects and rebuilds a venv
made the wrong way.

### `A module that was compiled using NumPy 1.x cannot be run in NumPy 2.x`

Something installed numpy ≥ 2 into the venv. Ubuntu 24.04 ships numpy 1.26 and
every ROS 2 Jazzy C extension — including the generated `two_towers.msg`
modules — is compiled against that ABI. `requirements.txt` caps numpy below 2
for exactly this reason; check nothing has upgraded past it:

```bash
./venv/bin/pip install -r requirements.txt
```

### Detector dies inside `YOLO()` under systemd but works by hand

ultralytics needs a writable config directory and somewhere to download
`yolov8n.pt`. The unit points `HOME` and `YOLO_CONFIG_DIR` at
`/var/lib/two-towers` via `StateDirectory=`; if you wrote your own unit, that
is what is missing.

### Detector dies with `Illegal instruction (core dumped)` / exit code −4

PyPI's default aarch64 torch wheel is the NVIDIA build (`+cu130`), compiled for
ARMv8.2+ Neoverse cores. The Pi 4's Cortex-A72 is ARMv8.0 — check with:

```bash
lscpu | grep -E "Model name|Flags"
```

No `asimddp` in the flags means a modern torch wheel will SIGILL on the first
matmul. `requirements.txt` pins `torch==2.2.2`, the last line whose aarch64
wheels are plain CPU builds targeting baseline ARMv8. Verify with:

```bash
./venv/bin/python -c "import torch; print(torch.__version__); a=torch.rand(64,64); print((a@a).sum().item())"
```

A version ending in `+cu` is the wrong wheel. Reinstalling needs an explicit
uninstall first — `pip install --index-url ...` will report "Requirement
already satisfied" and change nothing.

### Turret oscillates, or flips between tracking and scanning

Fixed by the dropout tolerance in `HasPersonDetection`. If you see `🎯 Track`
and `🔍 Scan` alternating within milliseconds, the tracker is treating a single
dropped detection frame as a lost target. Confirm `LOSS_THRESHOLD_FRAMES` is
present in `src/cpp/orthanc_tracker_node_bt.cpp` and that you rebuilt after
pulling.

### `ImportError: libGL.so.1: cannot open shared object file`

`opencv-python` links OpenGL, which Ubuntu **Server** does not ship:

```bash
sudo apt install -y libgl1 libglib2.0-0t64
```

Switching to `opencv-python-headless` looks like the tidier fix and is not:
ultralytics hard-depends on `opencv-python`, so the next
`pip install -r requirements.txt` reinstalls the GUI wheel over it.

### `fatal error: pigpio.h: No such file or directory`

Ubuntu 24.04 cannot provide a working pigpio from apt. There is no `pigpiod`
package on noble at all, and the `libpigpiod-if-dev` that *does* exist ships a
header including `pigpio.h`, which no noble package provides. Every guide
saying `apt install pigpio` is written for Raspberry Pi OS.

`scripts/bootstrap_pi.sh` builds pigpio from source for this reason. To do it
by hand:

```bash
git clone --depth 1 --branch v79 https://github.com/joan2937/pigpio.git /tmp/pigpio
cd /tmp/pigpio && make -j4 && sudo make install && sudo ldconfig
sudo install -m0644 ~/two-towers/deploy/systemd/pigpiod.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now pigpiod
```

Confirm the daemon is reachable with `systemctl is-active pigpiod && pigs t`
— that should print `active` and a tick count.

### Build fails at `find_package(behaviortree_cpp)`

```bash
sudo apt install ros-jazzy-behaviortree-cpp
```

There is no fallback path — the behavior tree node is the only tracker, so the
build fails at configure time rather than producing a package with no tracker
in it.

---

## Development Roadmap

### Phase 1: Single-Tower (Complete)
- [x] Behavior tree framework
- [x] Real-time YOLO detection
- [x] Proportional tracking control
- [x] ROS2 integration
- [x] Video streaming

### Phase 2: Multi-Tower (Planned)
- [ ] Deploy second turret hardware
- [ ] Tower status publishing
- [ ] Coordinator node for handoff
- [ ] Cooperative tracking mode

### Phase 3: Advanced Features (Future)
- [ ] Kalman filter motion prediction
- [ ] AI-powered decision making
- [ ] Multi-person tracking

---

## License

MIT License - See LICENSE file for details

---

## Author

**Tate Lloyd**
Robotics & Embedded Systems Engineer

- GitHub: [@tatelloyd](https://github.com/tatelloyd)
- LinkedIn: [/in/tatelloyd](https://www.linkedin.com/in/tatelloyd/)
- Email: tate.lloyd@yale.edu

---

*Built to demonstrate real-time autonomy, embedded control systems, and robotics software engineering.*
