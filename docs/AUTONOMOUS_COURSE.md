# 2026 Course Autonomous Navigation

## Goal and current scope

The autonomy baseline closes the loop from sensing to motion without using the
known Gazebo obstacle coordinates:

```text
front camera -> lane center estimate --+
                                       +-> local route scoring -> /cmd_vel
Hokuyo scan -> obstacle clearances ----+
odometry -> mission progress and finish detection
```

The deterministic 2026 course is a single closed lane, so its globally valid
route is defined by the observed lane. The planner therefore optimizes a local
route inside that lane instead of depending on a pre-built map. This preserves
the simulated-to-physical interface and continues to work when obstacles move.

This is a testable autonomous baseline, not proof of competition completion.
The final claim requires repeated Gazebo runs, measured completion rate,
collision count, time, path length, and lane error.

## Processing stages

### 1. Lane perception

`orange_lane_perception` converts front-camera pixels to a center pose in
`base_footprint`. The autonomous launch defaults to fixed-white segmentation
(threshold 145), uses the full upper image to see farther lane boundaries, and
excludes the bottom 30% of the image, where the simulated robot body otherwise
projects as a false lane boundary. The simulated front camera is pitched about
60 degrees down so lane tape remains in view beyond the robot's immediate
footprint. Adaptive HLS modes remain available for illumination testing:

- `mask_mode:=adaptive_hls` for local brightness adaptation;
- `mask_mode:=adaptive_hls_edges` to include edge candidates;
- one-boundary recovery when one tape line is occluded;
- temporal smoothing;
- confidence gating and diagnostics.

The output remains `/perception/lane_center` (`PoseStamped`), so another lane
model can replace the baseline without changing the controller.

### 2. Obstacle representation

`/hokuyo_scan` samples are converted to valid polar obstacle points. For each
candidate heading, the planner constructs a robot-width corridor and measures
the nearest obstacle projected along it. The configured 0.55 m half-width
includes the simulated 0.35 m chassis half-width and about 0.20 m margin.
In simulation, the Hokuyo is mounted ahead of the front mast so the scan does
not treat the robot's own mast as an obstacle.

### 3. Efficient local route selection

Nineteen candidate headings are evaluated from right to left. Candidates whose
short-horizon footprint would cross the configured lane-safe width are rejected.
For the remaining candidates, lower scores are better. The score combines:

- deviation from the detected lane-center direction;
- obstacle clearance;
- unnecessary turning;
- change from the previous selected heading.

With no obstacle, the robot follows the shortest lane-centered direction. With
an obstacle, it chooses the least-cost clear side and returns toward the lane
center after passing it. This is deterministic and directly inspectable in
`/autonomy/diagnostics`.

### 4. Motion and safety

The selected route is converted to differential-drive linear and angular
velocity. Speed decreases as clearance or perception confidence falls. Linear
and angular acceleration are limited to reduce oscillation.

The controller commands an immediate stop when:

- autonomy is disabled;
- lane, scan, or odometry input is missing or stale;
- the mission is complete.

When an obstacle is inside the hard stop distance, the robot rotates before it
advances. If every candidate corridor is blocked, linear velocity remains zero.

### 5. Completion detection

The 2026 baseline is a closed loop of approximately 92 m. The controller records
the first odometry pose as the start, requires the robot to leave the start
region, requires at least 70 m of accumulated valid motion, and then declares
completion when it returns within 2.5 m of the start. Odometry jumps above 1 m
per update are excluded from accumulated distance.

Completion is latched on `/autonomy/finished`, and the controller stops.
At completion, `/tmp/autonomy_run_summary.json` records elapsed time, travelled
distance, average speed, reference-path efficiency, minimum forward clearance,
near-stop events, and tracking/avoidance/blocked control-cycle counts.

## Build and launch

Inside the ROS 2 Humble environment:

```bash
colcon build --symlink-install --packages-up-to orange_autonomy
source install/setup.bash
ros2 launch orange_autonomy autonomous_course.launch.py autostart:=false
```

Starting disabled is recommended for the first visual inspection. In a second
ROS terminal, check the inputs and enable motion:

```bash
ros2 topic hz /perception/lane_center
ros2 topic hz /hokuyo_scan
ros2 topic echo /autonomy/diagnostics
ros2 service call /autonomy/set_enabled std_srvs/srv/SetBool "{data: true}"
```

To stop or reset:

```bash
ros2 service call /autonomy/set_enabled std_srvs/srv/SetBool "{data: false}"
ros2 service call /autonomy/reset_mission std_srvs/srv/Trigger "{}"
```

If Gazebo is already running, avoid spawning a duplicate robot:

```bash
ros2 launch orange_autonomy autonomous_course.launch.py \
  start_simulation:=false autostart:=false
```

## Validation sequence

1. Run with `autostart:=false` and verify lane, scan, odometry, and TF.
2. Validate the full interface for ten seconds:

   ```bash
   ros2 launch orange_sensor_tools sensor_contract_validation.launch.xml \
     profile:=simulation_autonomy
   ```

3. Enable autonomy at a low speed and confirm steering sign and emergency stop.
4. Run the fixed-white perception baseline and record completion, collision,
   elapsed time, path length, and lane-error metrics.
5. Repeat with `adaptive_hls` and `adaptive_hls_edges` under identical course
   conditions.
6. Tune only from recorded evidence, then run at least ten complete trials.
7. Repeat with shadows, partial lane occlusion, and legal obstacle variations.

## Main tuning parameters

Parameters are stored in `src/orange_autonomy/config/autonomy.yaml`.

| Parameter | Function |
| --- | --- |
| `max_linear_speed_mps` | Upper forward speed |
| `stop_distance_m` | Hard no-forward-motion distance |
| `slowdown_distance_m` | Distance where speed reduction begins |
| `corridor_half_width_m` | Chassis plus obstacle margin |
| `lane_weight` | Preference for the shortest lane-centered route |
| `clearance_weight` | Preference for routes farther from obstacles |
| `switching_weight` | Resistance to left-right oscillation |
| `lateral_gain` | Correction strength for lane-center offset |
| `heading_gain` | Correction strength for lane tangent |

Do not tune all parameters simultaneously. First verify sensor geometry and
steering sign, then safety distances, lane gains, route weights, and finally
speed and acceleration.
