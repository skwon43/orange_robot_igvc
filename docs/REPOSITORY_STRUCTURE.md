# Repository Structure

This repository contains two related runtimes: the supported ROS 2 simulation and the real-robot stack. They share message, TF, description, and algorithm interfaces, but they do not share every launch dependency.

## Ownership

```text
src/
  orange_description/   Robot Xacro, meshes, RViz configurations
  orange_gazebo/        Gazebo worlds, models, and simulation entry points
  orange_teleop/        Teleoperation nodes and launch files
  orange_sensor_tools/  C++ sensor processing nodes and sensor configs
  orange_bringup/       Real-robot drivers and localization utilities
  orange_navigation/     Navigation configuration and launch files
  orange_slam/           SLAM configuration and launch files
  serial/                Repository-owned serial ROS 2 dependency

deps/                   External dependency manifests
docker/                 Reproducible simulation image
compose.sim.yaml        Simulation container definition
docs/                   Specifications, checkpoints, and operational guidance
tools/                  Host-side helper scripts
```

The repository root is the colcon workspace, and all buildable ROS packages live under `src/`. The Compose file mounts the root at `/workspace`; colcon discovers the packages without changing their package names or installed interfaces.

## Supported Entry Points

The supported simulation path targets ROS 2 Humble and Gazebo Fortress:

- `orange_gazebo orange_sensor_demo.launch.xml`: complete sensor-demo checkpoint
- `orange_gazebo orange_igvc_baseline.launch.xml`: minimal simulator and bridge baseline

The following launch files are retained for compatibility with older robot or Gazebo Classic workflows. They are not part of the Fortress checkpoint and may require external packages that are not installed by the simulation Docker image:

- `empty_world.launch.xml`
- `orange_world.launch.xml`
- `orange_hosei.launch.xml`
- `orange_igvc.launch.xml`

Do not remove or rename compatibility launch files without checking downstream robot workflows. New simulation work should extend the two supported entry points above.

## Dependency Rules

- Package dependencies belong in `package.xml` and target-specific `find_package` / `ament_target_dependencies` declarations.
- External repositories listed in `deps/orange_ros2.rosinstall` are upstream dependencies, not repository-owned packages. Do not use that file to replace local packages with the same name.
- Runtime assets include `config`, `launch`, `firmware`, models, meshes, and RViz files. C++ source and private headers remain build inputs and are not installed as runtime data.
- `linefit_ground_segmentation_ros2` is an external dependency referenced by legacy processing launches; it is not vendored into this repository.

## Validation

Run these commands inside the simulation container after source changes:

```bash
source /opt/ros/humble/setup.bash
colcon list
colcon build --symlink-install --packages-up-to orange_gazebo orange_teleop
source install/setup.bash
ros2 launch orange_gazebo orange_igvc_baseline.launch.xml
ros2 launch orange_gazebo orange_sensor_demo.launch.xml
```

Use `docs/SIM_BASELINE.md` and `docs/SENSOR_DEMO_CHECKPOINT.md` for the expected environment and sensor contract.
