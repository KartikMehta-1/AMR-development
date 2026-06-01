# NUC Humble RViz Container

This profile is the NUC-side operator GUI for the Orin migration path.

Role:

```text
NUC
  - ROS 2 Humble GUI tools
  - RViz2
  - rqt_image_view
  - AMR workspace overlays for description/config files
  - DDS subscriber to topics published by the Orin runtime
```

Build:

```bash
cd /home/ubuntu/agent/repos/AMR-development
docker build -f docker/nuc-humble-rviz/Dockerfile -t amr/ros2-humble-rviz-nuc:amd64 .
```

Open RViz:

```bash
./scripts/open_amr_orin_rviz.sh
```

This profile is separate from the Foxy dev-PC/Nano workflow and should not be
used as proof that the legacy Foxy path still works.
