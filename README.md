# File-driven ROS 2 simulator

The simulator reads MATLAB scenarios directly with libmatio. Road geometry and
object motion come from the file; ego receives longitudinal velocity or
acceleration and lateral tire-angle commands on `/control/command/control_cmd`.
The optional internal lateral controller follows the supplied target path.

Longitudinal motion passes through a motor, torsional shaft and wheel model. The
existing speed controller still produces its feedback-only acceleration request;
motor torque and shaft force determine actual wheel force, while aerodynamic,
rolling, viscous and grade loads remain in the vehicle plant. Regenerative braking
is torque-limited and friction brakes supply any additional requested deceleration.
Relative shaft speed is damped by `shaft_damping` (default 120 N*m*s/rad), and
motor torque changes are limited by `motor_torque_rate_limit` (default 1000 N*m/s).
The controller acceleration request passes through a first-order low-pass filter
with `acceleration_command_time_constant:=0.05` s by default. Set the time
constant to zero to disable command filtering.
The simulator publishes motor state and cumulative drawn/recovered electrical
energy on `/simulator/powertrain_telemetry`. The visualizer adds these values to
`/vehicleTelemetryVisualization` in joules. The default illustrative efficiency
map is `config/motor_efficiency.csv`; replace it with measured motor data when
available.

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
tangent. `initial_displacement` places ego a distance in meters along the ordered
target path, interpolating position, elevation and tangent yaw; it defaults to
zero. For example, `initial_displacement:=120.0` starts 120 m along the route.
Explicit `initial_x` or `initial_y` still override the corresponding interpolated
coordinate, and explicit `initial_yaw` overrides the tangent. Displacements
beyond the target-path length are rejected. Initial speed defaults to zero.
Until the first
command, `use_scenario_speed_when_uncommanded=true` uses the local file speed
limit as the speed target. Set it to false when a planner must initiate motion.
After any command has been received, a missing/stale command targets zero speed
after `command_timeout` wall-clock seconds. External steering remains clamped by
`maximum_steering`. Scenario speed is a planning reference, not a clamp on
external commands.

## Speed-dependent local geometry range

Road corridors and pedestrian-crossing map features use a local cutoff computed
from the MAT speed limit at the nearest target waypoint:

```text
effective_scenario_distance = max(scenario_distance, target_speed * scenario_lookahead_time)
```

`scenario_lookahead_time` defaults to 6 s. At 130 km/h the range is about 217 m;
it does not collapse when the ego is stopped or moving slowly. `scenario_distance`
remains the minimum range (75 m by default). The existing along-corridor and
radial filters both use the resulting cutoff, which updates with the local MAT
speed limit. Object detection, occlusion shadows and prediction horizon retain
their separate `object_distance` and `prediction_horizon` settings.

```bash
ros2 launch simulator simulator.launch.py scenario_lookahead_time:=6.0
```

Set `scenario_lookahead_time:=0.0` to retain fixed-distance behavior. Both settings
are startup parameters; the effective distance changes automatically during playback.

## Occlusion and hypothetical actors

Occlusion and virtual actors are enabled by default. Rebuild `crp_msgs`,
`simulator`, `visualizer`, and any other `Scenario` consumers after this interface
change, and restart running consumers/bridges before mixing message layouts:

```bash
colcon build --packages-select crp_msgs simulator visualizer
source install/setup.bash
ros2 launch simulator simulator.launch.py initial_x:=65.0
```

This viewpoint places the parked obstacles and crossing in the default pedestrian
scene within range. Enable `/occlusionVisualization`, `/objectVisualization` and
`/objectPredictionVisualization` in the visualizer's Foxglove 3D panel.

Large vehicle/obstacle footprints (at least 2 m long, 1 m wide and 1 m high;
excluding pedestrians and cyclists) cast ego-view shadows out to `object_distance`.
Oriented rectangles are projected radially; shadow polygons exclude their source
footprint. An actor whose center is inside another visible occluder's shadow is
not published, including its future paths. Once its center leaves the shadow it
is published with `existence_probability=1.0`, independently of the source MAT
confidence or nonzero existence prior. Source existence zero still means absent.
Hidden occluders do not publish additional shadows or generate hypotheses.

`Scenario.occluded_regions` contains convex shadow polygons in `Scenario.header`'s
frame. Polygons may overlap. `Scenario.virtual_object_ids` identifies hypothetical
actors by UUID in `local_moving_objects`; consumers need not merge another object
array. UUIDs occupy a separate namespace from real detections and are keyed by
map feature and emergence boundary. These are recomputed hypotheses, not persistent
actor tracks, and never become ground-truth actors.

Hypotheses depend only on visible blockers and map geometry, not on hidden actors:

| Hypothesis | Existence | Conditional trajectory branches |
| --- | --- | --- |
| Pedestrian at an occluded crossing boundary | 0.5 | Walk at 0.8 m/s: 0.9; run at 3 m/s: 0.1. |
| Pedestrian stepping into an occluded roadside without a nearby crossing | 0.02 | Same walking/running mixture. |
| Vehicle emerging along an occluded lane | 0.5 | 15 m/s: 0.7; 8 m/s: 0.3. |

Crossing pedestrians follow the ordered crossing polyline in the emergence
direction; roadside pedestrians move from the shadowed road edge into the lane.
Vehicles follow the actual lane polyline and its traffic direction, including
curves. Predictions stop at the end of the available geometry. Seeds are placed
just inside an occluded-to-visible boundary, not in a visible blocker footprint.
Overlapping shadow intervals are merged and nearby duplicate hypotheses suppressed.
Fully hidden geometry with no visible emergence boundary produces no hypothesis,
as in the MATLAB reference. There are at most `maximum_virtual_objects=64` per
frame, with crossing pedestrians considered first.

Existence and path confidence are separate: joint probability is
`existence_probability * predicted_path.confidence`; branch confidences sum to one.
These priors are assumptions, not calibrated detection probabilities. Launch
parameters expose the three existence priors, the walking/running and vehicle
speeds, and their first-branch confidences. Set `generate_virtual_objects:=false`
to retain filtering/shadows without hypotheses, or `enable_occlusion:=false` to
publish all in-range real actors without shadows or hypotheses.

The reference is `scenario_files/predictiveCostMapAdvanced.m`, specifically
`getObjectPredictions`, `deleteUnseenObjects`, `generatePossibleStepInPoints`, and
`generatePossiblePredictedTrajectories`. Its actual pedestrian assignments are
0.8 m/s at 0.3 and 0.3 m/s at 0.7, despite comments saying 50/50. To reproduce that
mixture, set `pedestrian_walking_confidence:=0.3 pedestrian_running_speed:=0.3`.
The defaults instead implement the requested likely walking/less likely running
behavior. Roadside hypotheses and general lane directions extend the MATLAB
version, which only selects near-pi lane headings and has no roadside pedestrians.

Visibility is a base_link XY footprint approximation, matching the reference's
2D approach: center-based detection, no partial-body visibility, sensor-height
ray casting, terrain masking, or separation of stacked roads. Shadow display
heights use their source occluder's ground-contact height; they are not a terrain
mesh and may float/intersect sloped terrain. Existing real and virtual trajectories
retain their supplied/interpolated road elevation. Virtual speed branches do not
model traffic interactions or collision avoidance.

## Vehicle models

For the separate `pcm_planner` node, opt into direct acceleration commands:

```bash
ros2 launch simulator simulator.launch.py command_mode:=acceleration \
  vehicle_model:=dynamic use_internal_controller:=false \
  use_scenario_speed_when_uncommanded:=false
```

The default `command_mode=speed` is unchanged. Acceleration mode consumes
`Control.longitudinal.acceleration` before dynamic loads, using the same
`VehicleModel::stepAcceleration` integrator. It requests full braking before a
fresh command or after `command_timeout`. It does not fall back to scenario speed.
Never run the internal lateral controller when the planner commands steering.

Select the vehicle dynamics independently of the scenario:

```bash
ros2 launch simulator simulator.launch.py vehicle_model:=dynamic
```

`vehicle_model=kinematic` remains the default and preserves the previous
rear-axle kinematic integration and longitudinal speed controller. Dynamic mode
uses a planar, forward-only bicycle model whose pose and velocities refer to the
center of gravity (CG). Its low-speed kinematic limit uses that same CG reference,
including lateral velocity at the CG. The two modes do not have exactly the same
pose reference or turning trajectory; select one mode before starting a run.

The ROS command interface is unchanged:

```text
Control.longitudinal.velocity -> proportional speed controller
                             -> requested acceleration [m/s^2]
                             -> selected vehicle model
Control.lateral.steering_tire_angle -> steering input [rad]
```

`VehicleModel::stepAcceleration` is the acceleration-input integration API;
`VehicleModel::step` retains the speed-target convenience wrapper. Positive
requested acceleration represents propulsion force divided by vehicle mass,
not guaranteed net acceleration after loads. Negative values request braking.
Both are clamped by `maximum_acceleration` and `maximum_deceleration` before
dynamic loads are applied. The proportional speed controller has no integral
or load feed-forward, so resistance can leave a small steady-state speed error.
The optional internal lateral controller still supplies steering, and external
steering is used when `use_internal_controller=false`.

### Dynamic model

Let $u$ and $v$ be forward and lateral body velocities, $r$ the yaw rate, $m$ the
vehicle mass, $I_z$ the yaw inertia, $l_f$/$l_r$ the CG-to-axle distances, and
$\delta$ the front tire angle. Above the low-speed transition, the linear-slip
lateral tire forces are:

$$
F_{yf}=C_f\left(\delta-\frac{v+l_f r}{u}\right),\qquad
F_{yr}=C_r\left(\frac{l_r r-v}{u}\right).
$$

The lateral and yaw equations are:

$$
\dot v=\frac{F_{yf}\cos\delta+F_{yr}}{m}-ur,\qquad
\dot r=\frac{l_f F_{yf}\cos\delta-l_r F_{yr}}{I_z}.
$$

For forward motion, longitudinal integration includes steering-force projection,
body-frame coupling, aerodynamic drag, viscous resistance, rolling resistance,
and gravity along the road slope:

$$
\dot u=a_{request}+rv-\frac{F_{yf}\sin\delta}{m}
-\frac{\rho C_d A u^2}{2m}-\frac{b u}{m}
-C_{rr}g\cos\theta-g\sin\theta.
$$

Positive $\theta$ is uphill. At zero forward speed, rolling resistance does not
create backward motion or alternating force signs: propulsion must exceed the
opposing load to start moving. Forward speed is clamped at zero during braking;
reverse driving and uphill rollback are not simulated. A downhill load can start
forward motion if it exceeds braking and rolling resistance.

Below `kinematic_blend_speed`, dynamic mode uses the nonsingular CG kinematic
relationships $r=u\tan\delta/L$ and $v=l_r r$. Between that speed and
`dynamic_blend_speed`, a smoothstep weight blends the lateral/yaw state update
with the dynamic solution. Above the interval, the lateral/yaw equations use an
implicit 2x2 solve. Longitudinal loads remain active at low speed. Internal
substeps are no longer than `dynamics_integration_step`, independently of the ROS
publication period, to limit sensitivity to the timer step.

These are linear lateral-slip forces, without tire saturation, a friction
circle, load transfer, or roll/pitch suspension dynamics. Use the model in the
small-slip handling range, not as a calibrated limit-handling/drifting model.
There are no wheel-speed, torque, or longitudinal tire-slip states in this
version. Configuring an oversteering vehicle outside its stable speed range
can still produce physically unstable dynamics; the low-speed blend is not a
general-purpose stability controller.

### Parameters

| Launch parameter | Default | Meaning |
| --- | --- | --- |
| `vehicle_mass` | 1500 | Mass, kg. |
| `yaw_inertia` | 2500 | Yaw moment of inertia about CG, kg*m^2. |
| `wheelbase` | 2.7 | Axle separation, m. |
| `front_axle_distance` | 1.2 | CG-to-front axle, m; rear distance is wheelbase minus this. |
| `front_cornering_stiffness` | 80000 | Entire front axle stiffness, N/rad, not per tire. |
| `rear_cornering_stiffness` | 80000 | Entire rear axle stiffness, N/rad, not per tire. |
| `air_density` | 1.225 | kg/m^3. |
| `drag_coefficient` | 0.3 | Dimensionless. |
| `frontal_area` | 2.2 | m^2. |
| `rolling_resistance_coefficient` | 0.012 | Dimensionless. |
| `viscous_resistance` | 5.0 | Linear resistance, N*s/m. |
| `kinematic_blend_speed` | 0.5 | Pure CG-kinematic regime below this forward speed, m/s. |
| `dynamic_blend_speed` | 2.0 | Pure lateral-dynamic regime above this speed, m/s. |
| `dynamics_integration_step` | 0.005 | Maximum dynamic substep, s. |
| `road_pitch` | 0.0 | Constant test slope, rad; positive uphill. |
| `initial_lateral_speed` | 0.0 | Dynamic initial lateral body speed, m/s. |
| `initial_yaw_rate` | 0.0 | Dynamic initial yaw rate, rad/s. |

These are illustrative passenger-car defaults, not identified parameters for a
specific vehicle. Resistance coefficients can be zero to disable their terms.
Axle distances, mass, inertia and cornering stiffness must be positive; the
front axle distance must be less than the wheelbase and the upper blend speed
must exceed the lower one. Invalid configurations are rejected at startup.
Kinematic mode does not apply the new resistance/grade loads or dynamic initial
lateral/yaw states after integration, preserving its existing behavior.

With `use_road_gradient=true` (default), Nx3 target paths supply the dynamic
grade load automatically. The nearest XY target-path segment is used to
interpolate ego height; its dz/horizontal-distance slope, projected along ego's
yaw direction, supplies positive-uphill pitch. Reversing the heading reverses
the grade sign. Forward speed is measured along the inclined vehicle axis;
horizontal motion includes its cosine projection. Ego height is constrained
to the interpolated road after each update, and its pose/TF includes pitch.

For a 5% constant load-only uphill test, use `use_road_gradient:=false` with
`road_pitch:=0.049958`. The constant `road_pitch` is also the load fallback for
Nx2 files. It does not replace the actual road geometry or tilt a flat file.
Kinematic mode follows road elevation and pitch but does not apply grade loads.

This is a road-constrained model, not vertical suspension or banked-road dynamics.
Road height comes from the target route, including when ego is laterally offset;
there is no independently supplied cross-slope, bank, or terrain mesh. The nearest
XY projection does not distinguish stacked roads at identical XY locations.

Dynamic `/ego` messages now include lateral velocity in `twist.twist.linear.y`,
body-frame inertial lateral acceleration in `accel.accel.linear.y`, and yaw
acceleration in `accel.accel.angular.z`, as well as the existing forward speed
and yaw rate. Dynamic accelerations include rotating-frame corrections:
$a_x=\dot u-rv$ and $a_y=\dot v+ru$, averaged over the publication interval.

## MATLAB data contract

Save the fields as top-level MAT variables, as in the supplied generator's
`save(..., '-struct', 'quasiGlobalData')` call. No CSV conversion is required.

| Variable | Meaning |
| --- | --- |
| `time` | Uniformly spaced time samples in seconds, starting at zero. |
| `targetPath_qg` | Ordered Nx2 or Nx3 map-frame target waypoints, meters; Nx2 means z=0. |
| `lanes` | Cell array of ordered map-frame corridor centerlines; Nx3 for an elevated scenario. |
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
Elevated scenarios also require finite `position.z` for actors, `trajectories.z`
for alternative predictions, and crossing `z` when those records exist. Position
and crossing z may be scalar or match the x/y series; prediction z must cover the
same future rows and observation columns. Two-dimensional legacy scenarios retain
z=0 when those fields are absent. Missing required z, short coordinate arrays and
NaN/Inf elevations are rejected instead of silently filled or interpolated across
gaps. Optional `drivableSurface_qg` is also validated when present; lane topology
and geometry still come from `lanes`.
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

For elevated files, `/ego` includes map z and a pitch/yaw quaternion. Scenario
positions and orientations use the complete inverse ego transform, not just XY
rotation: target/road boundaries, objects, predictions and crossings retain their
relative height. Object and path orientations include tangent pitch; a one-point
prediction retains the object's current pitch. Pitch is positive uphill internally
and uses negative ROS Euler pitch to tilt the forward x axis upward. No roll/bank
or pitch-rate dynamics are modeled.
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
The independent `vehicle_model` test checks standstill and repeated stop/start
transitions, load equations and gradient signs, steady-turn behavior, yaw-inertia
transients, and substep consistency. `vehicle_model_ros` starts each model on an
isolated ROS domain, drives it through the existing velocity/steering command
topic, checks lateral/yaw publications and finite state, and verifies watchdog
braking to rest. The vehicle-model test itself does not require MAT fixtures.