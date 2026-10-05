# gary_the_snail

A ROS 2 package for autonomous control of a [BlueROV2](https://bluerobotics.com/store/rov/bluerov2/) underwater vehicle, developed for the MIT Beaver Works Summer Institute Autonomous Underwater Vehicles program.

The package implements closed-loop depth and heading stabilization, an AprilTag-based vision pipeline for detecting and pursuing another ROV, lane following from the downward camera, and a scripted open-loop maneuver routine. Vehicle I/O is handled through [`rosmav`](https://github.com/BWSI-UAV/rosmav), which bridges ROS 2 to the ArduSub flight controller over MAVLink.

---

## Architecture

Every node is small and single-purpose; they communicate entirely over ROS 2 topics. The design separates **sensing**, **control**, and **actuation** so that any high-level behavior (AprilTag pursuit, lane following, manual setpoints) can drive the vehicle by publishing setpoints without knowing anything about the thruster mixer.

```
                    ┌─────────────────────────────────────────┐
                    │          rosmav / ArduSub               │
                    │  bluerov2_hardware_interface + camera   │
                    └───┬─────────┬──────────┬────────▲───────┘
         /pressure      │         │ /heading │ /camera│ /manual_control
                        ▼         ▼          ▼        │ /override_rc
              ┌──────────────┐    │     ┌─────────────┴──────┐
              │ depth        │    │     │  Perception        │
              │ publisher    │    │     │  • april_tag       │
              └──────┬───────┘    │     │  • lane_following  │
               /depth│            │     └─────┬──────────────┘
                     ▼            ▼           │ setpoints
              ┌──────────────┐ ┌──────────────▼───┐
              │ depth_hold   │ │ heading_control  │
              │   (PID)      │ │     (PID)        │
              └──────┬───────┘ └────────┬─────────┘
            /depth_control      /heading_control
                      └────────┬─────────┘
                               ▼
                      ┌──────────────────┐
                      │   control node   │  ← /target_x, /target_y
                      │  (mixer, 20 Hz)  │
                      └────────┬─────────┘
                               │ /manual_control
                               ▼
                           thrusters
```

### Control layer

| Node | Executable | Role |
| --- | --- | --- |
| `DepthPublisher` | `depth_publisher` | Converts `FluidPressure` from the external barometer into metric depth via the hydrostatic relation `d = (P − P₀) / ρg`. |
| `DepthHold` | `depth_hold` | PID controller on depth error. Accepts absolute setpoints on `target_depth` and incremental offsets on `relative_depth`. Integral term is clamped and the output saturated to ±100. |
| `HeadingControl` | `heading_control` | PID controller on compass heading with angle wrapping to (−180°, 180°], so the vehicle always turns the short way around. Accepts `target_heading` and `relative_heading`. |
| `Control` | `control` | Mixer. Collects the depth and heading controller outputs plus lateral/forward setpoints (`target_x`, `target_y`) and publishes a single unified `ManualControl` message at 20 Hz. |

Keeping the mixer separate means the two PID loops never contend for the `manual_control` topic, and surge/sway can be commanded independently of the stabilized axes.

### Perception layer

**`april_tag`** — Runs the `dt_apriltags` detector (`tag36h11`, 5 cm tags) on the forward camera stream with full pose estimation from calibrated intrinsics. Tag IDs are partitioned into front-facing and rear-facing sets so the node can tell which side of the target vehicle it is looking at. From the tag's translation vector it derives a yaw bearing, converts that into an absolute heading setpoint relative to the current compass reading, and modulates forward speed with distance — closing in on the target, holding station inside 0.2 m, and triggering the lights on a confirmed close approach. Detecting the target's front face instead executes an evasive sequence: descend, drive through, reverse heading, and surface.

**`lane_following`** / **`lane_detection`** / `lane_following.py` — A classical CV pipeline for following lane markings on the pool floor. The frame is cropped to its lower half, passed through Canny edge detection and a probabilistic Hough transform, and the resulting line segments are deduplicated by slope and bottom-edge intercept. Surviving segments are paired into lanes by angular similarity, the lane nearest the image center is selected, and its midline drives a PID loop on lateral offset while the midline's slope drives a heading correction. If no lane is visible the node commands a yaw sweep to reacquire.

### Utility nodes

| Node | Executable | Role |
| --- | --- | --- |
| `ArmDisarm` | `arm_disarm_client` | Calls the `arming` service, holds the vehicle armed for a fixed window, then disarms cleanly. |
| `FlashlightControl` | `flashlight_control` | Maps a `Bool` on `flash` to RC override PWM on channels 9 and 10 (the BlueROV2 lumen lights), scaling a 0–100 level to 1000–2000 μs. |
| `Movement` | `movement_publisher` | Plays a scripted, time-parameterized maneuver sequence directly over `ManualControl`, with a braking pulse at the end of each move to cancel momentum. Used for the choreographed-routine demo. |
| `ImageSaver` | `image_save` | Dumps camera frames to disk as PNGs for offline calibration and tuning. |
| `TargetDepthPublisher` | `target_depth` | Interactive CLI for publishing depth setpoints during bench testing. |
| `TargetHeadingPublisher` | `target_heading` | Interactive CLI for publishing heading setpoints during bench testing. |

---

## Topic reference

| Topic | Type | Direction |
| --- | --- | --- |
| `pressure` | `sensor_msgs/FluidPressure` | from `rosmav` |
| `heading` | `std_msgs/Int16` | from `rosmav` |
| `camera` | `sensor_msgs/Image` | from `rosmav` |
| `depth` | `std_msgs/Float32` | `depth_publisher` → `depth_hold` |
| `target_depth` / `relative_depth` | `std_msgs/Float32` | → `depth_hold` |
| `target_heading` / `relative_heading` | `std_msgs/Int16` | → `heading_control` |
| `depth_control` / `heading_control` | `std_msgs/Float32` | PID → `control` |
| `target_x` / `target_y` | `std_msgs/Float32` | perception → `control` |
| `manual_control` | `mavros_msgs/ManualControl` | `control` → `rosmav` |
| `flash` | `std_msgs/Bool` | → `flashlight_control` |
| `override_rc` | `mavros_msgs/OverrideRCIn` | `flashlight_control` → `rosmav` |
| `arming` | `std_srvs/SetBool` | service on `rosmav` |

---

## Requirements

- ROS 2 (`ament_python` build type) and a configured workspace
- [`rosmav`](https://github.com/BWSI-UAV/rosmav) — BlueROV2 hardware and camera interfaces
- ArduSub-flashed vehicle, or the ArduSub SITL simulator
- Python: `rclpy`, `numpy`, `opencv-python`, `cv_bridge`, `dt_apriltags`

Declared ROS dependencies are in `package.xml`. The vision dependencies (`opencv-python`, `cv_bridge`, `dt_apriltags`) are not yet declared there and must be installed into the Python environment used to run the nodes.

> **Note on the virtualenv.** `setup.py` sets the console-script shebang to `~/.virtualenvs/ardusub/bin/python`. If your environment lives elsewhere, edit `virtualenv_name` or `executable_path` before building, or the installed executables will fail to launch.

---

## Build

From the root of your ROS 2 workspace:

```bash
git clone <this-repo> src/gary_the_snail
colcon build --packages-select gary_the_snail
source install/setup.bash
```

## Running

Three launch configurations are provided:

```bash
# Full stabilized stack, global namespace
ros2 launch gary_the_snail control.launch.yaml

# Same stack under the rov1 namespace (multi-vehicle / competition runs)
ros2 launch gary_the_snail land.launch.yaml

# Lane following only, against the simulator
ros2 launch gary_the_snail lane_follow_sim.yaml
```

`control.launch.yaml` and `land.launch.yaml` bring up the `rosmav` hardware and camera interfaces alongside `depth_hold`, `depth_publisher`, `heading_control`, `control`, and `flashlight_control`. The `april_tag`, `lane_follow`, and interactive setpoint nodes are commented out by default — uncomment the behavior you want for a given run, since the AprilTag and lane-following nodes both command the same axes and should not be active simultaneously.

Arm the vehicle separately when not using the launch file's arming node:

```bash
ros2 run gary_the_snail arm_disarm_client
```

---

## Tuning

Gains are set in the constructor of each controller node.

| Controller | Kp | Ki | Kd | File |
| --- | --- | --- | --- | --- |
| Depth hold | 60.0 | 8.0 | 50.0 | `gary_the_snail/bluerov2_depth_hold.py` |
| Heading | 0.70 | 0.20 | 0.20 | `gary_the_snail/bluerov2_heading_control.py` |
| Lane lateral | 60.0 | 8.0 | 50.0 | `gary_the_snail/bluerov2_lane_following.py` |

Both PID loops clamp the integral term to ±20 to limit windup during long saturated transients, and skip the derivative term on the first iteration to avoid a spike from an undefined `dt`.

Camera intrinsics for pose estimation are hardcoded in `bluerov2_april_tag.py` (`fx`, `fy`, `cx`, `cy`). Recalibrate and update these if the camera or its resolution changes — tag range estimates degrade quickly with stale intrinsics.

---

## Repository layout

```
gary_the_snail/
├── gary_the_snail/
│   ├── bluerov2_april_tag.py          # AprilTag detection and pursuit behavior
│   ├── bluerov2_arm_disarm.py         # arming service client
│   ├── bluerov2_control.py            # setpoint mixer → ManualControl
│   ├── bluerov2_depth_hold.py         # depth PID
│   ├── bluerov2_depth_publisher.py    # pressure → depth
│   ├── bluerov2_flashlight_control.py # light control via RC override
│   ├── bluerov2_heading_control.py    # heading PID
│   ├── bluerov2_image_save.py         # frame capture utility
│   ├── bluerov2_lane_following.py     # lane-following control node
│   ├── bluerov2_movement.py           # scripted maneuver sequence
│   ├── bluerov2_publish_target_*.py   # interactive setpoint CLIs
│   ├── lane_detection.py              # Canny + Hough line/lane extraction
│   └── lane_following.py              # lane center and steering recommendation
├── launch/
│   ├── control.launch.yaml
│   ├── land.launch.yaml
│   └── lane_follow_sim.yaml
├── test/                              # ament copyright, flake8, pep257 linters
├── package.xml
└── setup.py
```

## Testing

Standard `ament` linters:

```bash
colcon test --packages-select gary_the_snail
colcon test-result --verbose
```
