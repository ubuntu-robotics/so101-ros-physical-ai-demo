# SO-101 ROS Physical AI — Core Robot Stack

> ROS 2 Jazzy · ros2_control · MoveIt 2 · MoveIt Servo

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-blue?logo=ros)](https://docs.ros.org/en/jazzy/)
[![License: Apache 2.0](https://img.shields.io/badge/License-Apache%202.0-green.svg)](LICENSE)
[![MoveIt 2](https://img.shields.io/badge/MoveIt%202-Motion%20Planning-orange)](https://moveit.ros.org/)

ROS 2 (Jazzy) stack for a two-arm **SO-101 leader/follower teleoperation** rig.
The **leader** arm is moved by hand and the **follower** mirrors it. Both arms
use Feetech STS3215 servos driven by `feetech_ros2_driver` (a `ros2_control`
hardware interface, pulled in as a git submodule).

This repo covers the **core robot** packages only — description/URDF, bringup
launch orchestration, teleoperation, MoveIt 2 config, and the protective-stop
safety-enforcement subsystem. It intentionally does **not** include AI
perception (depth/vision models) or snap packaging — those live in sibling
repos:

- Perception (depth-based / vision-based protective-stop triggers, packaged
  as snaps): see the `ai-vision-ros2-demo` repo.
- Snap packaging of the packages in this repo (`so101-bringup`,
  `so101-teleop`, `so101-safety`): see the SO-101 snaps repo.

Those external perception snaps publish `/safety/protective_stop`
(`std_msgs/Bool`); `so101_safety`'s nodes in this repo only consume that
topic — they don't care who publishes it or why.

## Two mirroring implementations

Kept side by side, mutually exclusive at runtime (both drive the same
follower / serial port):

- **Direct teleop** (`teleop_split`): copies the leader's absolute joint
  positions straight to the follower. Snappy, no IK, no safety envelope of
  its own — relies on `so101_safety`'s `trajectory_safety_gate` to freeze the
  arm (via `arm_trajectory_controller`) on a protective stop.
- **MoveIt Servo** (`leader_servo_jog` → `JointJog`): drives the follower
  through MoveIt Servo for joint-limit/collision/singularity safety, output
  is an absolute position stream to `arm_forward_controller`. Pairs with
  `so101_safety`'s `safety_pause_bridge` (calls Servo's `pause_servo`
  service on a protective stop).

Joint order everywhere: `[shoulder_pan, shoulder_lift, elbow_flex, wrist_flex,
wrist_roll]` (+ `gripper` handled separately). Namespaces: `/leader`,
`/follower`.

## Packages

| Package | Role |
| --- | --- |
| `so101_description` | URDF/xacro, STL meshes, RViz configs. `variant:=leader\|follower`, `hardware_type:=mock\|real\|gazebo`. |
| `so101_moveit_config` | MoveIt 2 + MoveIt Servo config: SRDF, OMPL planning, `joint_limits.yaml`, `kinematics.yaml` (`pick_ik`), `so101_servo.yaml`, `servo.launch.py`. |
| `so101_bringup` | Launch orchestration, `ros2_control` controller YAMLs, hardware joint configs, camera configs, RViz, Gazebo worlds. |
| `so101_teleop` | Teleop C++ nodes: `teleop`, `teleop_split`, `leader_servo_jog`, `leader_sim_keyboard`. |
| `so101_safety` | Protective-stop enforcement: `trajectory_safety_gate` (direct-teleop path) and `safety_pause_bridge` (Servo path). Pure `ament_cmake`, no Python/ML deps — both nodes just subscribe to `/safety/protective_stop`. |
| `feetech_ros2_driver` | Feetech STS3215 `ros2_control` hardware interface (git submodule). |

## Hardware

- Two SO-101 arms (leader + follower) with Feetech STS3215 servos, connected
  via USB.
- Device numbering (`ttyACM0`/`ttyACM1`) is **not stable** across replug —
  set up udev rules for stable `/dev/so101_leader` / `/dev/so101_follower`
  symlinks (see [`docs/assets/99-so101.rules.example`](docs/assets/99-so101.rules.example)).
- Before running on real hardware you must complete the LeRobot motor setup +
  calibration steps for both arms (writes servo IDs and calibration values to
  EEPROM) — see the [LeRobot SO-101 guide](https://huggingface.co/docs/lerobot/so101).

> **Hardware safety:** restarting the follower's `ros2_control_node` runs the
> driver's `on_deactivate`, which **torque-offs all follower joints** — the
> arm goes limp and can drop under gravity. Support the arm before any
> follower restart; keep a physical e-stop nearby.

## Installation

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone --recurse-submodules https://github.com/ubuntu-robotics/so101-ros-physical-ai.git

cd ~/ros2_ws
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Usage

### Direct teleop (leader → follower, trajectory controller + safety gate)

```bash
ros2 launch so101_bringup teleop_split.launch.py \
  leader_usb_port:=/dev/so101_leader \
  follower_usb_port:=/dev/so101_follower
```

### Teleop via forward controller (no trajectory smoothing/freeze)

```bash
ros2 launch so101_bringup teleop.launch.py
```

Common overrides: `arm_controller:=trajectory_controller`,
`use_cameras:=false use_camera_tf:=false`, `use_teleop_rviz:=false`.

### MoveIt Servo teleop (simulation)

```bash
ros2 launch so101_bringup sim_servo_teleop.launch.py
```

### Gazebo simulation (direct teleop, two arms, keyboard-driven leader)

```bash
sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-gz-ros2-control
ros2 launch so101_bringup gazebo_teleop_sim.launch.py
```

Keyboard controls for the simulated leader:

| Keys | Joint | Direction |
| ---- | ----- | --------- |
| `1` / `q` | shoulder_pan | + / - |
| `2` / `w` | shoulder_lift | + / - |
| `3` / `e` | elbow_flex | + / - |
| `4` / `r` | wrist_flex | + / - |
| `5` / `t` | wrist_roll | + / - |
| `p` / `o` | gripper | open / close |
| Space | arm | reset arm joints to zero |

### MoveIt 2 (follower only)

```bash
ros2 launch so101_bringup follower_moveit_demo.launch.py
```

### Testing the safety-stop path without a perception snap

`so101_safety`'s nodes only consume `/safety/protective_stop`
(`std_msgs/Bool`) — you can exercise the freeze/pause behavior directly
without any AI perception running:

```bash
ros2 topic pub /safety/protective_stop std_msgs/msg/Bool "data: true" --once
# arm should freeze (trajectory_safety_gate) or Servo should pause (safety_pause_bridge)
ros2 topic pub /safety/protective_stop std_msgs/msg/Bool "data: false" --once
```

For the full end-to-end demo with a real perception source (depth or
object-detection based), see the `ai-vision-ros2-demo` repo.

## Requirements

- Ubuntu 24.04 + ROS 2 Jazzy
- `rosdep`, `colcon`
- (Optional) USB cameras for `so101_bringup/launch/cameras.launch.py`
- (Optional) `ros-jazzy-ros-gz-sim` / `ros-jazzy-ros-gz-bridge` /
  `ros-jazzy-gz-ros2-control` for Gazebo simulation

## Author

Dmitri Manajev

## License

Apache License 2.0 — see [LICENSE](LICENSE) for details.
