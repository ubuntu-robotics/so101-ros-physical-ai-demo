# SO-101 ROS Physical AI — Core Robot Stack — Agent Context

## What this project is

A ROS 2 (Jazzy) stack for a **two-arm SO-101 leader/follower teleoperation**
rig. The **leader** arm is moved by hand and the **follower** mirrors it.
Arms use Feetech STS3215 servos driven by `feetech_ros2_driver` (a
`ros2_control` hardware interface, git submodule).

This is the **core robot** repo: description/URDF, bringup, teleop, MoveIt 2
config, and protective-stop enforcement only. It does **not** contain AI
perception (depth/vision models) or any snap packaging:

- AI perception (publishes `/safety/protective_stop`, packaged as snaps) —
  separate `ai-vision-ros2-demo` repo.
- Snap packaging of `so101-bringup` / `so101-teleop` / `so101-safety` —
  separate SO-101 snaps repo. That repo's `snapcraft.yaml` colcon parts pull
  this repo's source via `source: https://github.com/ubuntu-robotics/so101-ros-physical-ai.git`
  (pinned to a branch/tag), rather than vendoring a copy of this code.

There are two mirroring implementations, kept side by side:
- **Direct teleop** (`teleop_split`): copies the leader's absolute joint
  positions to the follower. Snappy, no IK, no safety envelope of its own.
  This is the original, reliable path. Pairs with `arm_trajectory_controller`
  + `so101_safety`'s `trajectory_safety_gate`.
- **MoveIt Servo** (`leader_servo_jog` → JointJog): drives the follower
  through MoveIt Servo for joint-limit/collision/singularity safety. Output
  is an absolute position stream to `arm_forward_controller`. Pairs with
  `so101_safety`'s `safety_pause_bridge` (calls Servo's `pause_servo`
  service). Newer, still being tuned.

## Hardware (device numbering is NOT stable across replug)

- **Leader** arm: state-only, no command interfaces.
- **Follower** arm: position-commanded.
- Set up udev rules for stable `/dev/so101_leader` / `/dev/so101_follower`
  symlinks before relying on the launch file defaults — see
  `docs/assets/99-so101.rules.example`.

⚠️ **Hardware safety:** restarting the follower's `ros2_control_node` runs
the driver's `on_deactivate`, which **torque-offs all follower joints** → the
arm goes limp and can drop under gravity. Warn the user / have them support
the arm before any follower restart. Prefer a physical e-stop nearby.

## Package map

| Package | Role |
| --- | --- |
| `so101_description` | URDF/xacro, meshes. `variant:=leader\|follower`, `hardware_type:=mock\|real\|gazebo`. |
| `so101_moveit_config` | MoveIt + **Servo** config: `so101_servo.yaml`, `kinematics.yaml` (`pick_ik`), `joint_limits.yaml`, SRDF, `servo.launch.py`. |
| `so101_bringup` | Launch orchestration, ros2_control controller YAMLs, hardware joint configs, cameras, RViz, Gazebo worlds. |
| `so101_teleop` | Teleop C++ nodes: `teleop_split`, `teleop`, `leader_servo_jog`, `leader_sim_keyboard`. (Safety nodes live in `so101_safety`, not here.) |
| `so101_safety` | Safety subsystem: **enforcement-only** — `trajectory_safety_gate`/`safety_pause_bridge` (C++, pure `ament_cmake`, no Python/ML deps). Both just subscribe to `/safety/protective_stop` (`std_msgs/Bool`) and don't care who published it or why. The perception-driven trigger logic (ROI/thresholds/hysteresis) lives entirely in the separate `ai-vision-ros2-demo` repo's perception nodes — this repo has no perception/ML code at all. |
| `feetech_ros2_driver` | Feetech STS `ros2_control` hardware interface (git submodule, `ubuntu-robotics/feetech_ros2_driver`). |

Joint order everywhere: `[shoulder_pan, shoulder_lift, elbow_flex, wrist_flex,
wrist_roll]` (+ `gripper` handled separately). Namespaces: `/leader`,
`/follower`.

## Known gaps from the monorepo split

- `so101_bringup/launch/moveit_py_test.launch.py` has a comment referencing
  `pixi run bridge-3d-moveit` — informational only, not a hard runtime
  dependency, but the Pixi env it refers to isn't part of this repo either.

## Testing the safety-stop path without a perception snap

`so101_safety`'s nodes are perception-agnostic — exercise them directly:

```bash
ros2 topic pub /safety/protective_stop std_msgs/msg/Bool "data: true" --once
ros2 topic pub /safety/protective_stop std_msgs/msg/Bool "data: false" --once
```

## Gotchas / decisions already made (don't relitigate without reason)

- **Servo follows slowly / elbow stalls:** MoveIt Servo is closed-loop on the
  *measured* follower state, so it only ever commands ~one tick ahead of
  actual. Combined with the Feetech's weak position gain
  (`p_coefficient=16`, `d_coefficient=32`), small steps don't overcome
  gravity (worst on `elbow_flex`). Mitigated by `scale.joint: 3.0` in
  `so101_servo.yaml`. Proper fix = raise Feetech P / lower D in
  `so101_bringup/config/hardware/follower_joints.yaml` (EEPROM-written at
  driver configure; needs follower restart).
- **Gripper:** `gripper_controller` is a
  `forward_command_controller/ForwardCommandController` (a
  `/follower/gripper_controller/commands` `Float64MultiArray` topic). Both
  demos forward the leader gripper *position* there (`leader_servo_jog`
  directly; `teleop_split` with `gripper_mode:=forward_position`). It is NOT
  a GripperActionController.
- Servo runs in **JOINT_JOG** mode (pure joint space) — no IK/Jacobian in
  play for the mirror.
- **`arm_forward_controller` vs `arm_trajectory_controller`:** both are
  defined in `follower_split_controllers.yaml` but only one is spawned/used
  per demo.
  - `arm_forward_controller` (`forward_command_controller/ForwardCommandController`,
    `interface_name: position`) is a dumb pipe: whatever position array
    arrives on its `/commands` topic is written straight to hardware every
    cycle, no interpolation/timing/action API. Used by the **Servo** path
    because Servo itself already does the interpolation/rate-limiting/
    collision-checking.
  - `arm_trajectory_controller` (`joint_trajectory_controller/JointTrajectoryController`)
    takes `JointTrajectory`/`FollowJointTrajectory` goals, interpolates
    between waypoints, respects `joint_limits.yaml`, and exposes an action
    API (so `trajectory_safety_gate` can hold/freeze the last goal on
    protective stop). Used by the **split** (direct) path since
    `teleop_split` has no safety layer of its own and needs the controller
    to provide smoothing + a "freeze" mechanism.
- **Servo "velocity" input is converted to position output**, because the
  Feetech firmware/`arm_forward_controller` only accept position writes:
  `leader_servo_jog` publishes unitless `JointJog` (`command_in_type:
  unitless`) → `servo_node` integrates each tick (`cmd * scale.joint *
  publish_period`) against the *measured* follower position, runs it through
  the Butterworth smoother + collision/singularity checks, then emits an
  **absolute position** (`publish_joint_positions: true`,
  `publish_joint_velocities: false` in `so101_servo.yaml`) on
  `/follower/arm_forward_controller/commands`. Servo is effectively the
  integrator/interpolator standing in for what the JTC would normally do.

## Live debugging tips

- `ros2 control` CLI is NOT installed; query the controller_manager service
  directly (`/follower/controller_manager/list_controllers`).
- Servo health: `/follower/servo_node/status` (`code: 0` = OK).
- Watch the mirror: compare `/leader/joint_states` vs
  `/follower/joint_states`, and `/follower/servo_node/delta_joint_cmds` /
  `/follower/arm_forward_controller/commands`.
