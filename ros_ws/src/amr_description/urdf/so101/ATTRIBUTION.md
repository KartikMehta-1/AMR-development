# SO-101 Upstream Description Attribution

The SO-101 URDFs and STL mesh assets in this directory and in
`meshes/so101/assets` are vendored from:

```text
TheRobotStudio/SO-ARM100
https://github.com/TheRobotStudio/SO-ARM100/tree/main/Simulation/SO101
```

The upstream files are licensed under Apache-2.0. A copy of the upstream license
is stored as `LICENSE.Apache-2.0`.

Local AMR integration changes:

- Mesh paths are rewritten to `package://amr_description/meshes/so101/assets/...`.
- Link, joint, actuator, and transmission names are prefixed with `so101_` in
  `../so101_upstream.xacro` so the arm can coexist with the AMR base URDF.
- A fixed AMR mount, `so101_tool0`, and wrist webcam frames are added by the
  wrapper Xacro.
