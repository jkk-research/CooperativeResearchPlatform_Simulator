# Scenario generation

This package loads a MATLAB scenario, subscribes to `/ego`, and publishes
`/scenario` plus `/scenario_visualization`. It also publishes the reduced
`/simulator/tracking_scenario` stream used by the safety supervisor. Scenario
geometry and actor predictions remain in ego-relative `base_link`, preserving
the planner-facing message contract.

## Run

The simulator launch starts both the ego simulator and this package:

```bash
ros2 launch simulator simulator.launch.py
```

For a real vehicle or another ego source, run this package independently after
`/ego` is available:

```bash
ros2 launch scenario_generation scenario_generation.launch.py \
  scenario_file:=/absolute/path/to/scenario.mat
```

`initial_displacement` selects the ego's initial location along the ordered MAT
target path. Explicit `initial_x`, `initial_y`, or `initial_yaw` override its
local pose components.

## Frames and time

MAT coordinates are Cartesian coordinates in the fixed `map_local` frame. When
`/ego.header.frame_id` is `map_local`, its XY position and yaw are used directly.
When it is `map` or empty, the first ego pose is treated as map-framed and aligned
to the MAT initial pose using XY and yaw only. The package publishes a fixed
`map -> map_local` transform and, by default, a dynamic `map -> base_link`
transform from `/ego`. Set `publish_ego_tf:=false` if another node already
publishes that dynamic transform. The fixed alignment does not change as the
vehicle moves. Local elevation and pitch are derived from the MAT road geometry.

Scenario playback time is measured from the first `/ego.header.stamp`, so zero is
a valid first stamp and subsequent stamps should be monotonic. The generator
does not simulate or publish ego motion.