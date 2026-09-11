# Orange Robot IGVC Simulation Baseline

This document defines the reproducible simulation environment for the Orange Robot IGVC project.

The simulation environment is intentionally separated from the real-robot runtime. Simulation and hardware should expose compatible ROS 2 topics and TF interfaces so that higher-level autonomy software can be shared.

## Architecture Decision

### Simulation

- Host OS: Ubuntu 22.04 recommended
- Container runtime: Docker Engine + Docker Compose
- ROS distribution: ROS 2 Humble
- Simulator: Gazebo Fortress
- ROS/Gazebo integration: `ros_gz`
- GPU acceleration: NVIDIA GPU through NVIDIA Container Toolkit

### Real Robot

GPU-accelerated perception, including future Isaac ROS components, is treated as a separate real-robot runtime layer.

The common interface between simulation and the real robot should be standard ROS 2 topics and TF rather than direct GPU-memory coupling.

## Initial Simulation Milestone

The first baseline is intentionally minimal:

1. Build a reproducible ROS 2 Humble + Gazebo Fortress container.
2. Confirm NVIDIA GPU access from the container.
3. Launch the Orange IGVC simulation world.
4. Spawn the robot.
5. Send `/cmd_vel`.
6. Verify `/odom`, `/tf`, and `/joint_states`.

LiDAR processing, SLAM, Nav2, and higher-level autonomy will be added only after this baseline is stable.

## Tested Host Configuration

Initial development environment:

- Ubuntu 22.04.5 LTS (Jammy)
- Docker 29.7.2
- Docker Compose v5.3.1
- NVIDIA GeForce RTX 4060
- NVIDIA driver 610.43.02

These exact versions are not strict requirements. They record the first verified development environment.

## Verified Container Baseline

The first Docker image was successfully built on September 5, 2026 using:

```bash
docker build \
  -t orange-igvc-sim:humble \
  -f docker/sim/Dockerfile .
```

The container was launched with GPU access using:

```bash
docker run --rm -it \
  --gpus all \
  orange-igvc-sim:humble
```

The following runtime checks were verified inside the container:

```bash
echo $ROS_DISTRO
ros2 pkg list | grep ros_gz
ign gazebo --versions
nvidia-smi
```

Verified results:

- `ROS_DISTRO=humble`
- `ros_gz`
- `ros_gz_bridge`
- `ros_gz_image`
- `ros_gz_interfaces`
- `ros_gz_sim`
- `ros_gz_sim_demos`
- Ignition Gazebo `6.18.0`
- NVIDIA GeForce RTX 4060 visible from inside the container

The command `gz sim` is not available in this environment. Fortress uses the `ign gazebo` CLI. This is important because the current repository contains some newer `gz`-namespace simulator conventions that may need to be normalized for the Humble/Fortress baseline.

## Verified GUI and OpenGL Baseline

X11 forwarding from the Docker container to the Ubuntu desktop was verified using `xeyes`.

For the current simulation container, which runs as `root`, grant X11 access only to the local root user before launching a GUI container:

```bash
xhost +si:localuser:root
```

A direct Docker launch can then use:

```bash
docker run --rm -it \
  --gpus all \
  --network host \
  -e DISPLAY=$DISPLAY \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  orange-igvc-sim:humble
```

Inside the container:

```bash
xeyes
glxinfo -B
```

Verified OpenGL results:

- X11 display: `:1`
- Direct rendering: `Yes`
- OpenGL vendor: `NVIDIA Corporation`
- OpenGL renderer: `NVIDIA GeForce RTX 4060/PCIe/SSE2`
- OpenGL version: `4.6.0 NVIDIA 610.43.02`
- Dedicated video memory visible: `8188 MB`

This confirms that Gazebo and RViz GUI applications can use hardware-accelerated NVIDIA OpenGL from inside the simulation container.

After GUI use is complete, X11 access may be revoked with:

```bash
xhost -si:localuser:root
```

If Gazebo reports errors such as `Authorization required` or `qt.qpa.xcb: could not connect to display`, re-run the X11 grant above from the host before starting the GUI container.

## Docker Compose Workflow

The root-level `compose.sim.yaml` captures the verified GPU, host-network, host-IPC, X11, and workspace-mount configuration.

A reusable host-side helper is provided in `tools/igvc_shell.sh`. From the repository root:

```bash
source tools/igvc_shell.sh
```

Then use:

```bash
igvc-sim
```

for a normal interactive simulation shell, or:

```bash
igvc-gui
```

for an interactive shell with temporary X11 access. `igvc-gui` grants `root` access to the host X server before opening the container and revokes that access when the shell exits.

The helper automatically sources ROS 2 Humble and, when present, `/workspace/install/setup.bash` inside the container. It also determines the repository root from the helper script location, so users do not need to hard-code a local checkout path.

The equivalent manual workflow is:

```bash
xhost +si:localuser:root

docker compose -f compose.sim.yaml run --rm sim

xhost -si:localuser:root
```

Build or update the image with:

```bash
docker compose -f compose.sim.yaml build
```

The repository is mounted at `/workspace`, while colcon `build`, `install`, and `log` directories are stored in Docker named volumes.

The Compose workflow was verified on September 5, 2026. Running `glxinfo -B` inside a container started by Compose reported:

- Direct rendering: `Yes`
- OpenGL vendor: `NVIDIA Corporation`
- OpenGL renderer: `NVIDIA GeForce RTX 4060/PCIe/SSE2`
- OpenGL version: `4.6.0 NVIDIA 610.43.02`

Therefore the Compose path reproduces the same GPU-accelerated X11/OpenGL environment as the direct `docker run` configuration.

## Verified Workspace Discovery

Running `colcon list` from `/workspace` in the Compose container successfully discovered the eight ROS 2 packages currently stored in this repository:

```text
orange_bringup       src/orange_bringup       (ros.ament_python)
orange_description   src/orange_description   (ros.ament_cmake)
orange_gazebo        src/orange_gazebo        (ros.ament_cmake)
orange_navigation    src/orange_navigation    (ros.ament_cmake)
orange_sensor_tools  src/orange_sensor_tools  (ros.ament_cmake)
orange_slam          src/orange_slam          (ros.ament_cmake)
orange_teleop        src/orange_teleop        (ros.ament_python)
serial               src/serial               (ros.ament_cmake)
```

This confirms that the repository root can be used directly as the colcon workspace root; the packages do not need to be moved under an additional `src/` directory for the current baseline.

The full workspace is not yet treated as a single simulation dependency set. Real-robot packages and higher-level navigation packages declare additional hardware-specific and external dependencies. The simulation baseline will therefore validate a minimal simulation package subset before enabling the complete stack.

## Verified Simulation Dependencies

The simulation-core dependency set was checked with:

```bash
rosdep check \
  --from-paths \
    src/orange_description \
    src/orange_gazebo \
    src/orange_sensor_tools \
    src/orange_teleop \
    src/serial \
  --ignore-src \
  -r
```

After removing Gazebo Classic-only manifest dependencies, aligning the simulation packages with `ros_gz`, and removing the redundant ROS `joint_state_publisher` path from the IGVC simulation launch, the command completed with:

```text
All system dependencies have been satisfied
```

This is the dependency baseline for the first Orange IGVC simulation build. Hardware bringup and higher-level navigation dependencies remain intentionally outside this minimal baseline.

## Verified Simulation Build

The first simulation-core colcon build completed successfully on September 5, 2026:

```bash
colcon build \
  --symlink-install \
  --packages-up-to orange_gazebo orange_teleop
```

Five packages completed successfully:

```text
serial
orange_teleop
orange_sensor_tools
orange_description
orange_gazebo
```

The build produced only non-fatal developer/deprecation warnings from Boost and PCL/CMake.

## Verified Gazebo Launch

A minimal launch file, `orange_igvc_baseline.launch.xml`, is used to validate the simulator independently of higher-level perception, SLAM, and navigation nodes.

The baseline was launched with:

```bash
source install/setup.bash
ros2 launch orange_gazebo orange_igvc_baseline.launch.xml
```

Verified runtime results:

- Gazebo Fortress GUI opened successfully.
- The `orange_igvc` environment model loaded.
- The `orange_robot` entity was created successfully.
- `robot_state_publisher` started.
- `ros_gz_bridge` started for `/cmd_vel`, `/odom`, `/tf`, `/joint_states`, and `/clock`.
- The robot was visibly rendered in the IGVC world.

The current launch may emit non-fatal `XDG_RUNTIME_DIR`, EGL/DRI2, and URDF parser warnings. These do not block the verified baseline launch and will be cleaned up separately if they remain relevant.

## Verified Cross-Container ROS 2 Data Flow

A second Compose container could discover the ROS 2 bridge endpoints while the simulator ran in the first container, but initially received no message samples. Ignition Transport topics such as `/clock`, `/odom`, `/joint_states`, and `/tf` were present and actively publishing, which isolated the problem to the ROS 2 transport path between containers rather than the Gazebo systems themselves.

The Compose service was updated to share the host IPC namespace and to explicitly use ROS domain 0 by default:

```yaml
ipc: host

environment:
  ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-0}
```

After restarting both containers with this configuration, cross-container ROS 2 data flow was verified. In the second container:

```bash
source /workspace/install/setup.bash
echo $ROS_DOMAIN_ID
timeout 5 ros2 topic echo /clock --once
timeout 5 ros2 topic echo /odom --once
```

Verified results:

- `ROS_DOMAIN_ID=0`
- `/clock` delivered simulation time from Gazebo through `ros_gz_bridge` to the second container.
- `/odom` delivered an `nav_msgs/msg/Odometry` sample with `frame_id: odom` and `child_frame_id: base_footprint`.
- `/joint_states` delivered wheel joint positions and velocities for `left_wheel_hinge` and `right_wheel_hinge`.
- `/tf` delivered dynamic wheel transforms from `base_link` to `left_wheel` and `right_wheel`.

This rules out a ROS domain mismatch in the verified setup. The observed before/after behavior is consistent with an inter-container DDS shared-memory / IPC namespace issue; `ipc: host` is therefore part of the reproducible simulation baseline.

## Verified Command and Motion Feedback

The final baseline milestone was completed by publishing a forward velocity command on `/cmd_vel` from a second Compose container while Gazebo ran in the first container. The Orange robot moved forward visibly in the IGVC world, and the ROS 2 feedback topics remained active.

The tested command form was:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3}, angular: {z: 0.0}}"
```

During the manual test the command was allowed to run for an extended period, and the robot advanced roughly 12 m in the simulated environment. This observation is consistent with sustained forward motion and was not intended as a calibrated speed/distance test.

The simulator retains the last commanded velocity until another command is received, so a zero Twist should be sent explicitly to stop the robot:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

With this test, the initial simulation baseline is complete:

```text
Docker/GPU
  -> Gazebo Fortress launch
  -> Orange robot spawn
  -> ros_gz bridge
  -> cross-container ROS 2 data flow
  -> /cmd_vel command
  -> DiffDrive motion
  -> /odom, /joint_states, and /tf feedback
```

Sensor-processing, SLAM, Nav2, and higher-level autonomy remain intentionally outside this first baseline.

## Host Requirements

Recommended:

- Ubuntu 22.04
- Docker Engine
- Docker Compose
- NVIDIA driver
- NVIDIA Container Toolkit

Check the host environment:

```bash
docker --version
docker compose version
nvidia-smi
```

Verify GPU access from Docker:

```bash
docker run --rm --gpus all ubuntu nvidia-smi
```

A successful result should show the host NVIDIA GPU from inside the container.

## Repository Layout

```text
orange_robot_igvc/
├── compose.sim.yaml
├── docker/
│   └── sim/
│       └── Dockerfile
├── docs/
│   └── SIM_BASELINE.md
├── tools/
│   ├── generate_igvc_2026_course.py
│   └── igvc_shell.sh
├── deps/
│   └── orange_ros2.rosinstall
└── src/
  ├── orange_bringup/
  ├── orange_description/
  ├── orange_gazebo/
  ├── orange_navigation/
  ├── orange_sensor_tools/
  ├── orange_slam/
  ├── orange_teleop/
  └── serial/
```

Docker/runtime configuration is kept outside the ROS package tree.

## Known Gazebo Migration Issue

The current repository appears to contain an incomplete Gazebo migration.

Some simulation files use modern Gazebo / `ros_gz` conventions, while some package dependencies still reference the Gazebo Classic stack through `gazebo_ros_pkgs`.

The verified Humble container installs Gazebo Fortress / Ignition Gazebo 6.18.0. Therefore, newer simulator symbols such as `gz::sim` and plugin filenames using `gz-sim-*` should be treated as potential compatibility issues until validated against Fortress.

The `hyoon/sim-baseline-cleanup` branch will normalize this simulation stack before SLAM, Nav2, and other higher-level functionality are enabled.

The intended baseline is:

```text
Ubuntu 22.04
    └── Docker
         └── ROS 2 Humble
              └── Gazebo Fortress / Ignition Gazebo 6
                   └── ros_gz
```

## Development Branch

Simulation baseline development is performed on:

```text
hyoon/sim-baseline-cleanup
```

Changes should be validated locally in Docker before being proposed for merge into `main`.
