# Orange IGVC Project Progress Summary

**Date:** 2026-10-23
**Coverage:** Initial repository state through the current autonomous-driving
baseline

## Summary

The project has progressed from an existing collection of ROS 2 robot packages
and Gazebo assets to a reproducible 2026 IGVC simulation with a defined sensor
contract, quantitative lane-evaluation tools, and a first closed-loop autonomy
baseline.

The latest Gazebo run demonstrated autonomous motion, maintained lane output,
and obstacle passage. Approximately 13.6 m of travel was observed over about 70
seconds. A complete lap was not demonstrated because `/autonomy/finished` did
not become true. The current result should therefore be described as **working
partial autonomous navigation, with full-course completion still pending**.

## Project Progress

### Initial state

The repository already contained the Orange robot description, sensor and
motor packages, Gazebo assets, SLAM configuration, and legacy navigation
files. However, it did not provide:

- one reproducible ROS 2 Humble and Gazebo Fortress workflow;
- a machine-readable simulation-to-physical sensor interface;
- a deterministic 2026 course and lane ground truth;
- quantitative lane comparison across SIL, HIL, and physical trials; or
- a complete perception-to-`/cmd_vel` autonomous-driving path.

### Foundation completed through October 10

- Added the Docker-based ROS 2 Humble and Gazebo Fortress simulation workflow,
  including WSLg/X11 guidance.
- Added a deterministic 2026-normalized IGVC course with fixed lane and
  obstacle geometry for repeatable experiments.
- Added canonical ROS 2 topic, message, frame, rate, and TF contracts for the
  simulated and physical sensor interfaces.
- Added an automated contract validator and a physical-sensor mapping template.
- Added lane ground-truth export and lateral/heading error evaluation tools.
- Added the first front-camera lane-perception node, diagnostics, mask output,
  and evaluator-compatible prediction recorder.

### Main work completed after October 10

- Added fixed-white, adaptive-HLS, and edge-assisted lane candidate modes.
- Added one-boundary center recovery, temporal smoothing, and confidence-based
  lane output control.
- Added `orange_autonomy`, which combines lane pose, Hokuyo LaserScan, and
  odometry to select a safe local route and publish `/cmd_vel`.
- Added lane-boundary constraints, emergency stopping, sensor watchdogs,
  completion detection, and completed-run metric output.
- Added the `simulation_autonomy` interface-validation profile.
- Corrected ROS 2 Humble XML launch compatibility.
- Added top and bottom ROI controls to reduce missed distant tape and robot-body
  false positives.
- Changed the simulated front-camera pitch from approximately 74 degrees to 60
  degrees downward to increase useful forward ground visibility.
- Moved the simulated Hokuyo to `base_link` at `xyz="0.52 0 0.84"` so it no
  longer detects the robot's own mast as a near obstacle.

## Current Functional Pipeline

```text
Front camera -> lane center --------+
                                    +-> local route scoring -> /cmd_vel
Hokuyo scan -> obstacle clearance --+
Odometry -> progress and completion monitoring
```

The local planner evaluates nineteen candidate headings using lane alignment,
obstacle clearance, turning effort, command continuity, and lane-boundary
constraints. Missing or stale lane, scan, or odometry data causes the robot to
stop.

## Verification Status

| Item | Status |
| --- | --- |
| Seven-package ROS 2 build | Passed |
| Lane-perception tests | 7 passed |
| Autonomy and mission tests | 10 passed |
| Updated Python/XML syntax checks | Passed |
| Gazebo autonomous motion | Observed |
| Obstacle passage | Observed |
| Lane output during partial drive | Observed |
| Complete autonomous lap | Not yet verified |
| HIL and physical validation | Not yet performed |

The 17 regression tests were rerun after the latest ROI and camera changes and
all passed. One prior Gazebo command ended with exit code 137, but that process
exit is not treated as completion or failure evidence; ROS topics and runtime
diagnostics are evaluated separately.

## Current Limitations

1. Full-course completion and repeatability have not been demonstrated.
2. The fixed fractional ROI is a practical simulation correction; a calibrated
   ground-distance ROI and robot self-mask are still preferable.
3. The lane detector remains an image-processing baseline, not a trained
   semantic model, and requires quantitative shadow/lighting trials.
4. The simulated 60-degree camera pitch must be compared with measured physical
   mounting before SIL-to-HIL conclusions are made.
5. The green speed bump is not yet classified as traversable terrain and may be
   treated as an ordinary obstacle.
6. The current planner is a local planner for the single closed course, not a
   global planner for branching routes.

## Next Priorities

1. **Prove one complete deterministic lap.** Record a rosbag, passing autonomy
   contract report, lane CSV, `/autonomy/finished=true`, and completed-run JSON.
2. **Improve perception geometry.** Replace fractional ROI cropping with
   ground-distance filtering and a calibrated robot-body self-mask.
3. **Handle the speed bump.** Use depth or 3D geometry to distinguish a low
   traversable bump from a barrel or barricade, then slow down rather than
   avoid it.
4. **Run quantitative comparisons.** Compare fixed-white and adaptive modes
   over repeated trials using completion rate, collisions, time, path length,
   clearance, and lane-error metrics.
5. **Transfer to HIL and the physical robot.** Complete sensor mapping, measure
   camera/LiDAR extrinsics, validate the same ROS contract, and repeat the SIL
   metrics on the Jetson and physical platform.

## Reproduction

Inside the ROS 2 Humble environment:

```bash
colcon build --symlink-install --packages-up-to orange_autonomy
source install/setup.bash
ros2 launch orange_autonomy autonomous_course.launch.py autostart:=false
```

After verifying `/perception/lane_center`, `/hokuyo_scan`, and
`/autonomy/diagnostics`:

```bash
ros2 service call /autonomy/set_enabled std_srvs/srv/SetBool "{data: true}"
```

## Main Changed Areas

| Area | Purpose |
| --- | --- |
| `orange_gazebo` and `orange_description` | Reproducible course, robot, camera, and LiDAR simulation |
| `orange_sensor_tools` | Topic/TF contract, validation, and physical mapping |
| `orange_lane_perception` | Lane detection, projection, diagnostics, and metric recording |
| `orange_autonomy` | Local planning, obstacle avoidance, safety, completion, and run metrics |
| `tools/` | Course generation, ground-truth export, and lane-error evaluation |

## Conclusion

The project now has a reproducible simulation and a functioning closed-loop
autonomy baseline rather than only isolated sensor demonstrations. The next
research milestone is a fully archived collision-free lap, followed by repeated
quantitative SIL trials and matched HIL/physical validation.
