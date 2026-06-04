# AMR SO-101 MoveIt Config

Planning-only MoveIt 2 configuration for the single SO-101 arm mounted on the
AMR top plate.

This package intentionally does not define hardware execution. It is for
planning-scene, collision, named-pose, and RViz MotionPlanning bring-up.

Start the demo after installing MoveIt 2 packages:

```bash
ros2 launch amr_so101_moveit_config demo.launch.py
```

Current planning group:

- `so101_arm`: chain from `so101_base_link` to `so101_tool0`
- `so101_gripper`: gripper joint only

Before enabling execution:

- Measure the physical SO-101 mount transform on the AMR.
- Measure wrist camera extrinsics.
- Replace or simplify collision meshes if planning is slow or overly
  conservative.
- Add a real arm driver/controller only after supervised bench validation.
