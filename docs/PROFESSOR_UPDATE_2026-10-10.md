# Orange IGVC Project Update

**Date:** 2026-10-10
**Scope:** Simulation/reproducibility and sensors/interface/perception work

## Summary

The project has moved from a collection of ROS 2 robot packages to a
reproducible simulation baseline with a defined simulation-to-physical sensor
interface and an initial quantitative lane-perception workflow. Simulation and
hardware runtimes remain separate, but autonomy-facing ROS 2 topics, message
types, TF frames, and timing assumptions are now treated as one stable contract.

## Change From the Original Project State

### Original state

- The Orange ROS 2 packages, robot description, and Gazebo assets existed, but
  reproducible execution and validation boundaries were not fully specified.
- Sensor topics, frame IDs, image dimensions, and expected rates were not
  collected into a machine-readable interface contract.
- There was no automated simulation-to-physical sensor-interface validation or
  common lane-comparison format for SIL, HIL, and physical trials.

### Current state

- A Docker-based ROS 2 Humble and Gazebo Fortress baseline is documented with
  GPU/X11, host networking, host IPC, and WSLg guidance.
- A deterministic, normalized 2026 IGVC course provides fixed lane geometry
  and obstacle placement for repeatable experiments.
- ROS 2 sensor/TF contracts, physical mapping, lane ground truth, lane output,
  prediction recording, and quantitative evaluation are now defined.

## Delivered Work

### Simulation and reproducibility

- Docker Compose environment using ROS 2 Humble, Gazebo Fortress, and `ros_gz`.
- GPU/X11/WSLg helper workflow plus host IPC and a fixed ROS domain for
  cross-container ROS 2 data flow.
- Baseline launch and evidence for robot spawn, `/cmd_vel`, `/odom`, `/tf`, and
  `/joint_states`.
- Deterministic 2026 course generator and checked-in course model with 4.0 m
  nominal lane width and 0.0762 m tape width.

### Sensors and interface contract

- `sensor_interface_contract.yaml` covering command, odometry, TF, joint state,
  LiDAR, front/rear camera, ZED, and point-cloud interfaces.
- Profiles for baseline simulation, sensor demo, perception simulation, and the
  physical robot.
- `sensor_contract_validator` checks discovery, type, arrival, rate, image
  dimensions, frame IDs, odometry child frame, TF paths, and message gaps.
- A physical mapping template requires actual device identity, source topics,
  resolutions/rates, and measured mounting transforms.

### Perception and quantitative comparison

- Course centerline export plus CSV evaluation of mean absolute, RMS, p95, and
  maximum lateral error, with optional heading error.
- `orange_lane_perception` consumes front RGB, `CameraInfo`, and TF; projects
  white tape pixels to the ground; and publishes `/perception/lane_center` in
  `base_footprint`.
- Diagnostics and lane-mask topics support visual inspection. A recorder
  transforms valid lane estimates to `odom` and writes evaluator-compatible CSV.

## Alignment With the Reference Papers

- The 2027 ROS 2/Gazebo/HIL draft calls for reproducible SIL, followed by HIL
  and physical deployment, using a stable ROS 2 boundary. This update supplies
  the SIL baseline, contract, and common artifacts: contract report, rosbag or
  prediction CSV, and lane-metric JSON.
- The Beam/Cesium/AirSim reference calls for registered quantitative comparison
  rather than visual similarity alone. This update uses metric lane ground
  truth in `odom`, requires measured physical sensor extrinsics, and reserves
  physical lane survey/registration as a prerequisite for a valid comparison.

## Verification Status

### Completed

- Python syntax checks for the sensor validator and lane tools.
- XML parsing for modified manifests, launch files, and the generated SDF model.
- 212 paired course-boundary samples and 212 centerline samples exported.
- Zero error from an exact ground-truth prediction fixture.
- Lane-geometry unit tests for rotation, ground-plane intersection, valid
  lane-center estimation, and invalid-width rejection.
- Prior simulation baseline evidence for build, Gazebo launch, robot spawn,
  bridge topics, cross-container data flow, and `/cmd_vel` motion.

### Still required

1. Rebuild the image with the new OpenCV and `cv_bridge` dependencies.
2. Archive a passing `simulation_perception` contract report.
3. Inspect lane mask/diagnostics during a driven SIL trial and archive a rosbag,
   prediction CSV, and metric JSON.
4. Complete real-device mapping and measured extrinsics.
5. Repeat the same contract and metric workflow on the physical robot and the
   Jetson AGX Orin HIL configuration.
6. Survey or register the physical lane centerline before reporting physical
   lane error against the simulation reference.

## Conclusion

The project now has a research workflow rather than only a simulator demo:
reproducible simulator, stable sensor interface contract, physical mapping
boundary, perception output contract, and quantitative lane comparison. The
next phase is collecting matched SIL, HIL, and physical runtime evidence.

## References

- *SoutheastCon2027_IGVC_ROS2_Gazebo_HIL_Draft*: SIL/HIL/physical staging,
  stable ROS 2 interfaces, and timing/perception metrics.
- *SoutheastCon2024_Beam_Cesium_AirSim_SLAM*: registered quantitative
  simulation-to-physical comparison.
