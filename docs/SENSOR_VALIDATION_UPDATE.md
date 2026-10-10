# Sensor and Perception Validation Update

**Date:** 2026-10-09
**Scope:** Sensor integration, ROS 2 topic/frame validation, simulation-to-physical mapping, and quantitative lane/perception comparison.

## Summary

This update establishes an executable sensor-interface contract for the Orange
Robot IGVC simulation and defines the handoff boundary to the physical robot.
It also adds deterministic lane ground truth and reusable metric tools for
comparing SIL, HIL, and physical trials.

## Delivered Changes

### Canonical ROS 2 contract

`src/orange_sensor_tools/config/sensor_interface_contract.yaml` is the
authoritative contract for the public autonomy interface. It specifies topic
names, ROS message types, frame IDs, image dimensions, and target/minimum
publication rates for:

- `/cmd_vel`, `/odom`, `/tf`, and `/joint_states`;
- `/hokuyo_scan`;
- front RealSense-like RGB-D streams;
- rear RealSense-like RGB streams;
- ZED-like RGB-D streams; and
- the demo-only ZED depth point cloud.

The defined profiles are `simulation_baseline`, `simulation_demo`,
`simulation_perception`, and `physical`. The perception profile adds the lane
center output to the validated simulation interface. The physical profile
validates the same canonical external interface after physical driver remapping.

### Runtime contract validator

`sensor_contract_validator` is installed by `orange_sensor_tools` and launched
through `sensor_contract_validation.launch.xml`. During a timed validation
window it checks:

- topic discovery and message type;
- message arrival and observed publication rate;
- image dimensions;
- message header and odometry child frame IDs;
- `odom` to base and base to sensor TF availability; and
- maximum inter-message gap and estimated missed samples.

The validator produces a JSON report when `report_path` is provided. It does
not label clock differences as network latency because latency measurement
requires synchronized capture and receive clocks.

### Simulation-to-physical mapping

`physical_sensor_mapping.template.yaml` records the required inputs for the
front/rear RealSense devices, ZED, optional LiDAR, and base-state interface.
Before physical deployment it must be completed with the actual driver topic
names, device identities, and measured `base_link` to optical-frame mounting
transforms. Simulated mounting poses are not physical calibration values.

### Deterministic lane ground truth

The checked-in `orange_igvc_2026` model is generated from the existing course
topology with a 4.0 m lane width and 0.0762 m boundary-tape width. Both
supported simulation launches now default to `orange_igvc_2026.world`.

`tools/export_lane_ground_truth.py` exports the paired white-line boundaries as
a metric centerline CSV. `tools/evaluate_lane_perception.py` compares lane
predictions in `odom` coordinates with that centerline and reports mean
absolute, RMS, p95, and maximum lateral error plus optional heading error.

### ROS 2 lane-perception baseline

The new `orange_lane_perception` package provides an inspectable baseline for
the first quantitative SIL run. It subscribes to the canonical front RGB image
and `CameraInfo`, resolves the live optical-frame transform, projects white
lane-tape pixels onto the local ground plane, and fits a lane-center pose at a
fixed lookahead. The node publishes:

- `/perception/lane_center` as `geometry_msgs/msg/PoseStamped` in
  `base_footprint`;
- `/perception/lane_diagnostics` with per-frame detection validity and point
  counts; and
- `/perception/lane_mask` for visual debugging.

Its paired recorder transforms each valid local lane pose to `odom` and writes
the exact CSV schema consumed by `tools/evaluate_lane_perception.py`. The
detector is deliberately a transparent validation baseline, not a claim of
competition-ready lane perception. Any later model-based replacement must keep
the published topic, message type, and frame contract.

## Verification Completed

- Python syntax validation passed for the validator and both lane tools.
- XML parsing passed for the modified launch files, package manifest, and the
  generated 2026 SDF model.
- The deterministic course contains 212 inner and 212 outer lane-boundary
  links, and its centerline export contains 212 samples.
- A ground-truth prediction fixture produced zero lateral and heading error.
- Docker build was reported successful after Docker Desktop startup; the five
  simulation packages completed successfully.
- Python syntax validation passed for the new lane-perception package.
- Four dependency-free unit tests passed for quaternion rotation, ground-plane
  projection, valid lane-center estimation, and lane-width rejection.
- The new package manifest was parsed successfully. The updated YAML profile
  was manually reviewed; formal ROS-container parsing remains a runtime check.

## Runtime Notes

The initial Windows-host GUI attempt could not open Gazebo, RViz, or the
keyboard teleoperation terminal because no `DISPLAY` connection was available.
This is a host graphics integration issue, not a package-build failure. Use an
X11/WSLg display connection before GUI validation.

After the sensor demo is running, execute:

```bash
source /workspace/install/setup.bash
ros2 launch orange_sensor_tools sensor_contract_validation.launch.xml \
  profile:=simulation_demo \
  report_path:=/tmp/igvc-sim-contract.json
```

Build the new perception package and start it in a second shell after the
sensor demo is running:

```bash
colcon build --symlink-install --packages-up-to orange_lane_perception
source install/setup.bash
ros2 launch orange_lane_perception lane_perception.launch.py \
  output_path:=/tmp/lane_predictions.csv
```

## Remaining Work

1. Rebuild the Docker image with `cv_bridge` and OpenCV, then run and archive a
   passing `simulation_perception` contract report.
2. Inspect `/perception/lane_mask` and diagnostics while driving the
   deterministic 2026 course; tune the baseline only if it misses valid tape.
3. Record a simulation rosbag and lane-prediction CSV, then archive the metric
   JSON from the deterministic 2026 course.
4. Complete the physical mapping file with actual device drivers and measured
   extrinsics.
5. Run the physical contract validation and record lane predictions using the
   same public topics and CSV schema.
6. Execute the autonomy stack on the Jetson AGX Orin in HIL and collect the
   equivalent timing, rate, and lane-metric reports.
7. Register or survey the physical course centerline before interpreting
   physical lane error against the simulation ground truth.
