# AMR SO-101 MoveIt Config

MoveIt 2 configuration for the single SO-101 arm mounted on the AMR top plate.

By default this package starts in planning/fake-motion mode. Hardware execution
is available through the conservative `amr_so101_driver` trajectory bridge, but
it should remain wrist-roll-only by default. All-six execution is available only
for explicit supervised bring-up/calibration until physical joint limits, torque
margins, and collision constraints are updated.

Start the demo after installing MoveIt 2 packages:

```bash
ros2 launch amr_so101_moveit_config demo.launch.py
```

Start with the SO-101 bridge in fake mode:

```bash
ros2 launch amr_so101_moveit_config demo.launch.py \
  use_so101_driver:=true \
  use_joint_state_gui:=false \
  driver_use_fake_hardware:=true
```

When running together with the AMR base, start the base stack with its wheel
joint states remapped away from the global `/joint_states` topic:

```bash
ros2 launch amr_description orin_hardware.launch.py joint_states_topic:=/amr/joint_states
```

Then the SO-101 driver merger combines `/amr/joint_states` and
`/so101/joint_states` back into `/joint_states` for RViz and MoveIt.

Start the real wrist-roll-only bridge:

```bash
ros2 launch amr_so101_moveit_config demo.launch.py \
  use_so101_driver:=true \
  use_joint_state_gui:=false \
  driver_use_fake_hardware:=false \
  so101_port:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A7A058493-if00 \
  driver_allowed_joints:=so101_wrist_roll
```

Start the real all-six bridge only for supervised calibration:

```bash
ros2 launch amr_so101_moveit_config demo.launch.py \
  use_so101_driver:=true \
  use_joint_state_gui:=false \
  driver_use_fake_hardware:=false \
  so101_port:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A7A058493-if00 \
  driver_allowed_joints:=so101_shoulder_pan,so101_shoulder_lift,so101_elbow_flex,so101_wrist_flex,so101_wrist_roll,so101_gripper
```

Current planning group:

- `so101_arm`: chain from `so101_base_link` to `so101_tool0`
- `so101_gripper`: gripper joint only

Before enabling execution:

- Measure the physical SO-101 mount transform on the AMR.
- Measure wrist camera extrinsics.
- Calibrate physical joint limits. The recorded `home`/`carry`,
  `look_from_height`, and `ready_to_pick_up` poses came from encoder readings on
  the physical arm, and some values are outside the starter MoveIt/URDF limits.
- Replace or simplify collision meshes if planning is slow or overly
  conservative.
- Keep all-six execution supervised until limits and torque margins are updated.
