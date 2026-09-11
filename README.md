# Orange Robot IGVC — TNTech Simulation

This repository contains the Orange Robot ROS 2 stack together with the TNTech IGVC simulation workflow.

The current simulation baseline targets:

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Fortress
- Docker Compose
- NVIDIA GPU acceleration
- X11 GUI forwarding for Gazebo, RViz, and image tools

Simulation and real-robot runtimes are intentionally kept separate while sharing compatible ROS 2 topic and TF interfaces.

## Simulation Quick Start

From the repository root, load the helper functions:

```bash
source tools/igvc_shell.sh
```

Then open a simulation shell with temporary X11 access:

```bash
igvc-gui
```

The helper grants X11 access only to the local root user used by the current simulation container and revokes that access when the shell exits. This avoids the common Gazebo startup error:

```text
Authorization required, but no authorization protocol specified
qt.qpa.xcb: could not connect to display
```

For a non-GUI simulation shell, use:

```bash
igvc-sim
```

### First Build or After Source Changes

Inside the simulation container:

```bash
cd /workspace

colcon build \
  --symlink-install \
  --packages-up-to orange_gazebo orange_teleop

source install/setup.bash
```

## Verified Sensor Demo Checkpoint

The current student-shareable checkpoint is launched with:

```bash
ros2 launch orange_gazebo orange_sensor_demo.launch.xml
```

This single launch prepares:

- Gazebo Fortress with the IGVC course and Orange robot
- automatic third-person Gazebo follow camera
- keyboard teleoperation in a separate xterm window
- RViz with the robot model and Hokuyo LaserScan
- ZED-like forward RGB-D sensing and ROS-side depth point cloud
- front downward RealSense-like RGB-D sensing
- rear downward RealSense-like RGB sensing
- front RealSense RGB and ZED RGB image displays in RViz

The camera layout follows the current TNTech IGVC Competition sensor roles:

```text
Front RealSense-like:
  pose  = (0.42, 0.0, 1.48), pitch 1.2915 rad
  RGB-D = 424x240 @ 15 Hz
  depth = 0.10 to 2.50 m

Rear RealSense-like:
  pose = (-0.42, 0.0, 1.00), pitch 1.2915 rad, yaw pi
  RGB  = 424x240 @ 6 Hz

ZED-like forward camera:
  pose  = (0.43, 0.0, 1.58), pitch 0.43 rad
  RGB-D = 1280x720 @ 15 Hz
  depth = 0.20 to 5.00 m
```

Primary ROS topics include:

```text
/hokuyo_scan

/camera_front/color/image_raw
/camera_front/color/camera_info
/camera_front/depth/image_raw
/camera_front/depth/camera_info

/camera_rear/color/image_raw
/camera_rear/color/camera_info

/zed/zed_node/rgb/image_rect_color
/zed/zed_node/rgb/camera_info
/zed/zed_node/depth/depth_registered
/zed/zed_node/depth/camera_info
/zed/zed_node/depth/points
```

The ZED depth point cloud is reconstructed with `depth_image_proc` from the bridged depth image and CameraInfo so that the PointCloud2 follows the ROS optical-frame convention.

## Minimal Baseline Launch

For the lower-level simulator / bridge baseline without the additional RViz and demo helpers:

```bash
ros2 launch orange_gazebo orange_igvc_baseline.launch.xml
```

## Detailed Documentation

See:

- [`docs/REPOSITORY_STRUCTURE.md`](docs/REPOSITORY_STRUCTURE.md) - package ownership, supported entry points, and restructuring rules

- [`docs/SENSOR_DEMO_CHECKPOINT.md`](docs/SENSOR_DEMO_CHECKPOINT.md) — frozen September 5, 2026 student-shareable sensor-demo checkpoint
- [`docs/SIM_BASELINE.md`](docs/SIM_BASELINE.md) — reproducible Docker / ROS 2 / Gazebo baseline and validation history
- [`docs/IGVC_2026_COURSE_SPEC.md`](docs/IGVC_2026_COURSE_SPEC.md) — deterministic 2026 IGVC course specification
- [`docs/ORANGE_ROS2_UPSTREAM.md`](docs/ORANGE_ROS2_UPSTREAM.md) — upstream Orange ROS 2 package documentation

## Development

Current simulation sensor development is performed on:

```text
hyoon/sim-sensor-contract
```

Simulation changes should be validated in Docker before being proposed for merge into the main project branch. Higher-level lane detection, obstacle processing, NVBlox, SLAM, Nav2, and autonomous course execution should build on the verified sensor interface rather than silently changing it.
# orange_robot_igvc
