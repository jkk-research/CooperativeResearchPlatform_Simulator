# File-driven ROS 2 simulator

The simulator reads MATLAB scenarios directly with libmatio. Road geometry and
object motion come from the file; ego remains controlled by
`/control/command/control_cmd` using the longitudinal velocity and lateral tire
angle. The optional internal lateral controller follows the supplied target path.

## Build and run

Install the system dependency with `sudo apt install libmatio-dev`, or resolve the
package dependencies through rosdep. From the ROS workspace, after sourcing ROS:

```bash
colcon build --packages-select simulator
source install/setup.bash
ros2 launch simulator simulator.launch.py
```

The three MAT files in the sibling `scenario_files` directory are copied into the
installed package during the build. To select another scenario:

```bash
ros2 launch simulator simulator.launch.py \
  scenario_file:=/absolute/path/to/roundabout_with_conflicting_traffic_costmap_data.mat \
  initial_speed:=10.0 use_internal_controller:=true
```

Use `ros2 launch simulator simulator.launch.py --show-args` for all settings.
All simulator settings are startup settings. The old `/simulator/...` parameter
names and procedural object/road generator are replaced by the launch arguments
in `launch/simulator.launch.py`. Previously declared actuator-delay and
perception-error settings were unused and are not retained.

The default ego position and yaw come from the first target waypoint and its
tangent. Override `initial_x`, `initial_y` and `initial_yaw` separately; `auto`
retains the file-derived default. Initial speed defaults to zero. Until the first
command, `use_scenario_speed_when_uncommanded=true` uses the local file speed
limit as the speed target. Set it to false when a planner must initiate motion.
After any command has been received, a missing/stale command targets zero speed
after `command_timeout` wall-clock seconds. External steering remains clamped by
`maximum_steering`. Scenario speed is a planning reference, not a clamp on
external commands.

## MATLAB data contract

Save the fields as top-level MAT variables, as in the supplied generator's
`save(..., '-struct', 'quasiGlobalData')` call. No CSV conversion is required.

| Variable | Meaning |
| --- | --- |
| `time` | Uniformly spaced time samples in seconds, starting at zero. |
| `targetPath_qg` | Ordered Nx2 map-frame target waypoints, meters. |
| `lanes` | Cell array of ordered Nx2 map-frame corridor centerlines. |
| `drivableSurface_qg` | Concatenation of `lanes`; topology is taken from `lanes`, not inferred from the concatenation. |
| `speedLimit` | Scalar or one value per target waypoint, already in m/s. |
| `objects` | Vehicle struct array; defaults to UNKNOWN unless classification is provided. |
| `pedestrian` | Pedestrian struct array. |
| `cyclists` | Bicycle struct array. |
| `obstacles` | Static obstacle struct array; defaults to UNKNOWN classification. |
| `pedestrian_crossing` | Optional struct array with x/y walking centerlines and optional width in meters. |

Each actor has `position.x`, `position.y`, optional `velocity` (numeric series or
`velocity.velocity`), and optional `size = [length, width, height]` in meters.
Positions use the common `time` channel. Extra position samples beyond `time` are
ignored. Actors disappear after their position series ends. With
`trim_zero_padding=true`, trailing `(x=0, y=0, speed=0)` samples are treated as
generator padding for moving actors; disable this when an actor genuinely ends
its motion stopped at the map origin. Short optional series are extended with
their last value, including the short pedestrian confidence tail in the example.
Missing dimensions default to a 0.6 m pedestrian cylinder or a 1 m bounding box.
An optional scalar `classification` overrides the default using Autoware labels.
Absent velocity defaults to zero. Obstacles are expected to have constant poses.

The original `egoOrientation` field is not used: the supplied generator computes
`atan(diff(y)./diff(y))`, producing NaNs on straight roads and incorrect headings
elsewhere. Headings are derived using waypoint tangents and `atan2` instead.
`qg` positions are used directly as `map`; no additional origin transform is applied.

### Independent confidences

- Optional actor `existence_probability`: scalar or time series in [0, 1],
  published as `PredictedObject.existence_probability`. Defaults to 1.0 because
  the example exports do not include independent detection uncertainty.
- Actor `confidence`: scalar or time series for the **main trajectory**, not
  detection confidence. Defaults to 1.0.
- `objectPossibleTrajectories(branch).trajectories.x/y`: matrices indexed by
  `[absolute future time index, observation time index]`.
- `objectPossibleTrajectories(branch).confidence`: matrix with the same indexing.
  The diagonal value at the observation time supplies that branch's scalar weight.
  Future confidence values are used to detect the valid prediction extent, but
  are not published as per-point confidence.

The main trajectory and all valid alternatives are published together. Their
weights are normalized to sum to one, conditional on existence, without
multiplying by existence probability. A zero total weight falls back to the main
trajectory. For example, existence 0.5 and branch weights 0.3/0.6/0.1 remain
separate fields with exactly those probabilities. The example generator's raw
weights do not generally sum to one, so normalization changes their absolute
values while retaining their ratios. Classification probability is independent
and defaults to 1.0.

For independent detection uncertainty, add it explicitly when exporting:

```matlab
quasiGlobalData.objects(1).existence_probability = ...
    0.5 * ones(size(quasiGlobalData.time));
```

## Published data and timing

- `/ego`: map pose, vehicle-frame forward speed/acceleration, yaw rate and tire angle.
- `/scenario`: road geometry and predicted objects in `base_link`, sharing ego's
  publication timestamp. Nested headers are populated too.
- `paths[0]`: target route; `paths[1:]`: visible drivable-surface corridors.
- `paths[0].traffic_rules.maximum_speed`: speed limit at the nearest target waypoint.
  Target path points also carry their individual `longitudinal_velocity_mps` limits.
  Other corridors carry the current target-route limit because the exports do not
  supply per-lane speed limits.
- `local_moving_objects`: vehicles, pedestrians and cyclists with stable IDs.
- `local_obstacles`: static obstacles, also with constant-pose predictions.

Centerlines are expanded into boundaries perpendicular to their tangents using
`lane_width` and `target_path_width`. These widths are assumptions: the MAT files
do not contain actual road-edge geometry. No occupancy grid is synthesized;
`free_space`, right-of-way rules and `costPrimitive` are not populated from these files.
Pedestrian crossing geometry is published in `map_features.pedestrian_crossings`
using the scenario frame; absent MAT widths use the `crossing_width` launch
argument. `MapFeatures` groups typed static map geometry independently of paths.
Future map feature types belong in this container rather than as new top-level
scenario fields. Feature IDs must be unique across feature types within a map
and remain stable between updates.

Path-specific `traffic_rules.applicable_map_feature_ids` can reference those IDs
without duplicating geometry. The supplied MATLAB data does not contain explicit
path-to-crossing associations, so the simulator leaves these lists empty. An
empty list means no associations were supplied, not that nearby crossings are
irrelevant. Association alone does not imply a stop or yield obligation.

The first update is scenario time zero. Ego integration uses `simulation_step`;
the wall timer uses `simulation_step / real_time_factor`. Object playback uses
the latest file sample at or before elapsed simulation time (sample-and-hold,
not interpolation). Predictions use the file's time step and stop at the selected
`prediction_horizon` or the end of valid trajectory data. When the integration
step differs from the file step, predictions are anchored to that latest file
sample. Use matching steps for exact alignment. Simulation does not publish
`/clock`; ROS headers use the node clock. By default publication stops at the file
end and state remains frozen. With `stop_at_scenario_end=false`, ego keeps moving
and object playback holds its last available file sample.

## Checks

With the sibling example files present:

```bash
colcon test --packages-select simulator --event-handlers console_direct+
colcon test-result --verbose
```

The focused test covers all three MAT schemas, lane topology, prediction matrix
indexing, probability separation/normalization, frame rotation, corridor width,
speed-limit mapping, prediction timing, and basic ego controller behavior.