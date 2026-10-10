# Sensor Interface Contract and Validation

This document defines the simulation-to-physical boundary for perception work.
Simulation drivers and physical drivers may differ, but autonomy nodes consume
the same canonical ROS 2 topics, message types, frames, and timing assumptions.

## Canonical Contract

The authoritative machine-readable contract is:

```text
src/orange_sensor_tools/config/sensor_interface_contract.yaml
```

`simulation_baseline` validates the Gazebo bridge. `simulation_demo` adds the
ROS-side ZED depth point cloud. `physical` validates the same public contract
after real driver topics have been remapped to the canonical names.

The contract currently requires these frame paths:

```text
odom -> base_footprint
base_link -> camera_front_optical_frame
base_link -> camera_rear_optical_frame
base_link -> zed_left_camera_optical_frame
base_link -> hokuyo
```

The canonical camera interfaces are:

```text
/camera_front/color/image_raw       424x240 RGB, 15 Hz target
/camera_front/depth/image_raw       424x240 depth, 15 Hz target
/camera_rear/color/image_raw        424x240 RGB, 6 Hz target
/zed/zed_node/rgb/image_rect_color  1280x720 RGB, 15 Hz target
/zed/zed_node/depth/depth_registered 1280x720 depth, 15 Hz target
```

## Runtime Validation

Start the supported sensor demo, then run the validator in a second shell.

```bash
ros2 launch orange_gazebo orange_sensor_demo.launch.xml

ros2 launch orange_sensor_tools sensor_contract_validation.launch.xml \
  profile:=simulation_demo \
  report_path:=/tmp/igvc-sim-contract.json
```

The validator fails when a required topic is missing, its type differs, no
sample arrives, a header frame differs, an image size differs, the observed
rate is below the configured minimum, or a required TF path is unavailable.
The JSON report also records observed rate, maximum inter-message gap, and an
estimated missed-sample count inferred from gaps. It does not claim to measure
network latency because that requires synchronized source and receive clocks.

## Physical Sensor Mapping

Before physical validation, copy
`physical_sensor_mapping.template.yaml` to a local
`physical_sensor_mapping.yaml` and complete it with:

1. device model and serial number;
2. actual driver namespace and source topics;
3. measured `base_link` to optical-frame translation and rotation;
4. confirmed image resolution and rate.

Driver implementation details remain behind this boundary. Configure driver
remappings or a relay so published messages use the canonical topic names and
their headers use the canonical optical frame. Publish the corresponding static
TF from the measured mounting transform. Do not reuse the simulated pose as a
physical calibration value.

Run the same validator on the physical robot:

```bash
ros2 launch orange_sensor_tools sensor_contract_validation.launch.xml \
  profile:=physical \
  report_path:=/tmp/igvc-physical-contract.json
```

## Lane and Perception Comparison

Quantitative lane results use the deterministic 2026 Gazebo course as the SIL
ground truth. Both supported simulator launches now default to
`orange_igvc_2026.world` so the test course is stable across runs.

Export its centerline in the world frame. The course include in
`orange_igvc_2026.world` has a `+13.0 m` Y offset.

```bash
python3 tools/export_lane_ground_truth.py \
  --offset-y 13.0 \
  --output /tmp/igvc_2026_centerline.csv
```

For every SIL, HIL, and physical trial, write one prediction CSV in `odom`
coordinates with these required columns:

```text
timestamp_ns,predicted_center_x_m,predicted_center_y_m
```

`predicted_heading_rad` is optional. Then calculate the comparable metrics:

```bash
python3 tools/evaluate_lane_perception.py \
  --ground-truth /tmp/igvc_2026_centerline.csv \
  --predictions /tmp/lane_predictions.csv \
  --label sil-baseline \
  --output /tmp/sil_lane_metrics.json
```

The output contains mean absolute lateral error, RMS lateral error, 95th
percentile lateral error, maximum lateral error, and optional heading error.
Use identical course geometry, time interval, and prediction schema in SIL,
HIL, and physical tests. Physical trials require a surveyed or otherwise
registered equivalent lane centerline before this metric is meaningful.

## Lane Perception Baseline

`orange_lane_perception` supplies an intentionally transparent starting point
for perception validation. It thresholds white tape in the front RGB image,
uses `CameraInfo` and the live `base_footprint <- camera_front_optical_frame`
TF to project image pixels onto the local ground plane, fits both lane
boundaries, and publishes a center pose:

```text
/perception/lane_center      geometry_msgs/msg/PoseStamped, base_footprint
/perception/lane_diagnostics diagnostic_msgs/msg/DiagnosticArray
/perception/lane_mask        sensor_msgs/msg/Image
```

The pose reports the centerline at a fixed 1.5 m lookahead. It is a baseline
for interface and metric work, not a final competition detector. It must be
re-tuned or replaced only behind the same output topic and frame contract.

Start it after the sensor demo. The recorder transforms each valid local pose
to `odom` and writes exactly the prediction CSV accepted by the evaluator.

```bash
ros2 launch orange_gazebo orange_sensor_demo.launch.xml

ros2 launch orange_lane_perception lane_perception.launch.py \
  output_path:=/tmp/lane_predictions.csv

ros2 launch orange_sensor_tools sensor_contract_validation.launch.xml \
  profile:=simulation_perception \
  report_path:=/tmp/igvc-perception-contract.json
```

After a representative driven run, stop the recorder and calculate the lane
metrics using the same ground-truth export command shown above. The prediction
CSV is a trial artifact and must not be committed as source data.
