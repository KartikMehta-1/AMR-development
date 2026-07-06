# AMR SO-101 Driver

This package provides the first hardware execution bridge between MoveIt2 and
the AMR-mounted SO-101 arm.

The bridge exposes:

- `/so101_arm_controller/follow_joint_trajectory`
- `/so101/joint_states`
- `/joint_states` from the optional merger node

It is intentionally conservative for initial bring-up:

- `use_fake_hardware` defaults to `true`.
- Real hardware mode validates all six expected STS3215 motor IDs before use.
- Only `so101_wrist_roll` is allowed to execute by default. All-six execution
  can be enabled explicitly for supervised bring-up/calibration.
- Unknown joints, large start mismatches, and large per-point jumps are rejected.
- `/so101/free_servos` releases torque through a `std_srvs/Trigger` service for
  manual pose recording.
- Torque is disabled on shutdown or execution failure.

For combined AMR base + SO-101 operation, use one source of truth for the global
joint state stream:

- AMR base publishes wheel joints to `/amr/joint_states`.
- SO-101 bridge publishes arm joints to `/so101/joint_states`.
- `amr_joint_state_merger` publishes the combined stream to `/joint_states`.

The combined `/joint_states` stream is what `robot_state_publisher`, RViz, and
MoveIt should consume.

Planning-only fake bridge:

```bash
ros2 launch amr_so101_moveit_config demo.launch.py \
  use_so101_driver:=true \
  use_joint_state_gui:=false \
  driver_use_fake_hardware:=true
```

Real wrist-roll-only bridge:

```bash
ros2 launch amr_so101_moveit_config demo.launch.py \
  use_so101_driver:=true \
  use_joint_state_gui:=false \
  driver_use_fake_hardware:=false \
  so101_port:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A7A058493-if00 \
  driver_allowed_joints:=so101_wrist_roll
```

Real all-six bridge for explicit supervised bring-up:

```bash
ros2 launch amr_so101_moveit_config demo.launch.py \
  use_so101_driver:=true \
  use_joint_state_gui:=false \
  driver_use_fake_hardware:=false \
  so101_port:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A7A058493-if00 \
  driver_allowed_joints:=so101_shoulder_pan,so101_shoulder_lift,so101_elbow_flex,so101_wrist_flex,so101_wrist_roll,so101_gripper
```

Current status: all-six teleop has moved the physical SO-101 under supervision.
The next required calibration is physical joint limits. Some encoder-captured
safe poses exceed the starter URDF/MoveIt limits, so release servos, record
safe min/max for each joint, and update URDF, MoveIt, and teleop limits with
margin before using named poses for autonomous grasping.
