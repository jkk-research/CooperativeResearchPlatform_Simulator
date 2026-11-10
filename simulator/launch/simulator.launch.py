from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


PARAMETERS = {
    'scenario_file': (
        PathJoinSubstitution([
            FindPackageShare('scenario_generation'), 'scenario_files',
            'roundabout_with_conflicting_traffic_costmap_data.mat',
        ]), str, 'MAT scenario file; qg coordinates are interpreted in map_local.'),
    'simulation_step': ('0.05', float, 'Ego integration step and publication period in simulation seconds.'),
    'simulation_time_stamps': ('false', bool, 'Stamp ego/scenario using elapsed simulation time; enabled by the split planner/tracker launch.'),
    'real_time_factor': ('1.0', float, 'Simulation seconds per wall-clock second.'),
    'stop_at_scenario_end': ('true', bool, 'Freeze simulation and stop publication at the final sample.'),
    'trim_zero_padding': ('true', bool, 'Remove trailing zero-position/zero-speed padding from moving actors.'),
    'scenario_distance': ('75.0', float, 'Minimum radius and along-corridor distance of published road geometry in meters.'),
    'scenario_lookahead_time': ('6.0', float, 'Road publication range is max(scenario_distance, MAT speed limit times this value); zero keeps fixed range.'),
    'object_distance': ('150.0', float, 'Maximum published object distance from ego in meters.'),
    'lane_width': ('3.85', float, 'Full width of each drivable-surface corridor in meters.'),
    'target_path_width': ('3.85', float, 'Full width of the target-path corridor in meters.'),
    'crossing_width': ('3.0', float, 'Pedestrian-crossing width in meters when absent from the MAT file.'),
    'prediction_horizon': ('4.0', float, 'Maximum object prediction horizon in seconds.'),
    'enable_occlusion': ('true', bool, 'Hide actor centers behind large oriented object footprints and publish shadows.'),
    'generate_virtual_objects': ('true', bool, 'Generate geometry-based possible actors in occluded areas.'),
    'crossing_pedestrian_probability': ('0.75', float, 'Existence prior for a possible pedestrian at an occluded crossing.'),
    'roadside_pedestrian_probability': ('0.02', float, 'Existence prior for a possible roadside pedestrian without a crossing.'),
    'virtual_vehicle_probability': ('0.5', float, 'Existence prior for a possible vehicle emerging along an occluded lane.'),
    'pedestrian_walking_speed': ('0.8', float, 'Likely pedestrian speed along the crossing or into the road, m/s.'),
    'pedestrian_running_speed': ('3.0', float, 'Less likely pedestrian speed, m/s.'),
    'pedestrian_walking_confidence': ('0.9', float, 'Conditional walking probability; the other branch has one minus this.'),
    'virtual_vehicle_fast_speed': ('15.0', float, 'First possible lane-following vehicle speed, m/s.'),
    'virtual_vehicle_slow_speed': ('8.0', float, 'Second possible lane-following vehicle speed, m/s.'),
    'virtual_vehicle_fast_confidence': ('0.7', float, 'Conditional first vehicle branch probability.'),
    'maximum_virtual_objects': ('64', int, 'Maximum geometry-based hypotheses published in one scenario.'),
    'initial_x': ('auto', float, 'Initial map_local x in meters, or auto for the first target waypoint.'),
    'initial_y': ('auto', float, 'Initial map_local y in meters, or auto for the first target waypoint.'),
    'initial_yaw': ('auto', float, 'Initial map_local yaw in radians, or auto for the target-path tangent.'),
    'initial_displacement': ('0.0', float, 'Initial distance in meters along the ordered target path; explicit initial_x/y still override its position.'),
    'initial_speed': ('13.0', float, 'Initial ego forward speed in m/s; default 50 km/h.'),
    'initial_acceleration': ('0.0', float, 'Initial ego longitudinal acceleration in m/s^2.'),
    'initial_lateral_speed': ('0.0', float, 'Initial body lateral velocity in m/s, dynamic model only.'),
    'initial_yaw_rate': ('0.0', float, 'Initial yaw rate in rad/s, dynamic model only.'),
    'use_internal_controller': ('true', bool, 'Follow targetPath_qg internally; otherwise use commanded steering.'),
    'use_scenario_speed_when_uncommanded': ('true', bool, 'Use speedLimit until the first external control command.'),
    'command_timeout': ('1.0', float, 'Wall-clock timeout after a command, then target zero speed.'),
    'command_mode': ('speed', str, 'Control.longitudinal input: speed or requested acceleration; stale commands brake.'),
    'wait_for_first_control_command': ('true', bool, 'Hold initial state and scenario time until the first command in acceleration mode.'),
    'vehicle_model': ('dynamic', str, 'Vehicle integration model: kinematic or dynamic.'),
    'wheelbase': ('2.7', float, 'Front-to-rear axle distance in meters for either model.'),
    'vehicle_mass': ('1500.0', float, 'Dynamic model vehicle mass in kg.'),
    'yaw_inertia': ('2500.0', float, 'Dynamic model moment of inertia about the CG vertical axis in kg*m^2.'),
    'front_axle_distance': ('1.2', float, 'CG-to-front axle distance in meters; rear distance is wheelbase minus this.'),
    'front_cornering_stiffness': ('80000.0', float, 'Total front axle linear cornering stiffness in N/rad.'),
    'rear_cornering_stiffness': ('80000.0', float, 'Total rear axle linear cornering stiffness in N/rad.'),
    'air_density': ('1.225', float, 'Air density in kg/m^3; zero disables aerodynamic drag.'),
    'drag_coefficient': ('0.3', float, 'Dimensionless aerodynamic drag coefficient.'),
    'frontal_area': ('2.2', float, 'Vehicle aerodynamic frontal area in m^2.'),
    'rolling_resistance_coefficient': ('0.012', float, 'Dimensionless rolling resistance coefficient.'),
    'viscous_resistance': ('5.0', float, 'Linear speed-dependent resistance coefficient in N*s/m.'),
    'use_road_gradient': ('true', bool, 'Derive dynamic-model gradient load from Nx3 target-path elevation.'),
    'road_pitch': ('0.0', float, 'Constant load-only road slope in radians when automatic gradient is disabled or the file is 2D.'),
    'kinematic_blend_speed': ('0.5', float, 'Below this forward speed in m/s use CG kinematics in dynamic mode.'),
    'dynamic_blend_speed': ('2.0', float, 'Above this forward speed in m/s use full dynamic lateral equations.'),
    'dynamics_integration_step': ('0.005', float, 'Maximum dynamic-model substep in seconds.'),
    'motor_efficiency_map': ('', str, 'CSV motor efficiency map; defaults to the installed simulator config.'),
    'motor_gear_ratio': ('9.0', float, 'Motor-to-first-shaft unitless gear ratio.'),
    'wheel_gear_ratio': ('1.0', float, 'Second-shaft-to-wheel unitless gear ratio.'),
    'wheel_radius': ('0.32', float, 'Effective tire radius in meters.'),
    'motor_inertia': ('0.02', float, 'Motor-side rotational inertia in kg*m^2.'),
    'wheel_inertia': ('1.2', float, 'Wheel-side reflected rotational inertia in kg*m^2.'),
    'motor_viscous_friction': ('0.001', float, 'Motor-side viscous rotational friction in N*m*s/rad.'),
    'wheel_viscous_friction': ('0.002', float, 'Wheel-side viscous rotational friction in N*m*s/rad.'),
    'shaft_stiffness': ('18000.0', float, 'Equivalent torsional shaft stiffness in N*m/rad.'),
    'shaft_damping': ('120.0', float, 'Equivalent torsional damping in N*m*s/rad.'),
    'maximum_motor_torque': ('200.0', float, 'Positive motor torque limit in N*m.'),
    'maximum_regenerative_torque': ('120.0', float, 'Regenerative torque limit in N*m.'),
    'motor_torque_rate_limit': ('1000.0', float, 'Maximum absolute motor torque change in N*m/s.'),
    'powertrain_integration_step': ('0.001', float, 'Maximum actuator and longitudinal-plant integration step in seconds.'),
    'acceleration_command_time_constant': ('0.05', float, 'First-order low-pass time constant for the controller acceleration request; zero disables filtering.'),
    'maximum_acceleration': ('9.81', float, 'Requested propulsion acceleration limit in m/s^2, before dynamic loads.'),
    'maximum_deceleration': ('9.81', float, 'Requested braking acceleration magnitude in m/s^2, before dynamic loads.'),
    'maximum_steering': ('0.6', float, 'Absolute front tire steering limit in radians; less than pi/2.'),
    'speed_gain': ('6.0', float, 'Proportional speed-controller gain in 1/s.'),
    'lookahead_time': ('1.5', float, 'Internal lateral-controller lookahead time in seconds.'),
    'minimum_lookahead': ('2.0', float, 'Minimum internal lateral-controller lookahead distance in meters.'),
}

SCENARIO_ONLY_PARAMETERS = {
    'scenario_distance', 'scenario_lookahead_time', 'object_distance', 'lane_width',
    'target_path_width', 'crossing_width', 'enable_occlusion', 'generate_virtual_objects',
    'crossing_pedestrian_probability', 'roadside_pedestrian_probability',
    'virtual_vehicle_probability', 'pedestrian_walking_speed', 'pedestrian_running_speed',
    'pedestrian_walking_confidence', 'virtual_vehicle_fast_speed', 'virtual_vehicle_slow_speed',
    'virtual_vehicle_fast_confidence', 'maximum_virtual_objects',
}
SHARED_PARAMETERS = {
    'scenario_file', 'trim_zero_padding', 'prediction_horizon', 'initial_x', 'initial_y',
    'initial_yaw', 'initial_displacement',
}


def launch_nodes(context):
    simulator_parameters = {}
    scenario_parameters = {}
    for name, (_, value_type, _) in PARAMETERS.items():
        if name in ('initial_x', 'initial_y', 'initial_yaw'):
            if LaunchConfiguration(name).perform(context).strip().lower() == 'auto':
                continue
        parameter = ParameterValue(LaunchConfiguration(name), value_type=value_type)
        if name not in SCENARIO_ONLY_PARAMETERS:
            simulator_parameters[name] = parameter
        if name in SCENARIO_ONLY_PARAMETERS or name in SHARED_PARAMETERS:
            scenario_parameters[name] = parameter
    return [
        Node(
            package='simulator', executable='simulator', name='simulator',
            output='screen', parameters=[simulator_parameters],
        ),
        Node(
            package='scenario_generation', executable='scenario_generator', name='scenario_generator',
            output='screen', parameters=[scenario_parameters],
        ),
        Node(
            package='simulator', executable='tf_broadcaster', name='tf_broadcaster',
            output='screen', condition=IfCondition(LaunchConfiguration('publish_tf')),
        ),
    ]


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, (default, _, description) in PARAMETERS.items()
    ]
    arguments.append(DeclareLaunchArgument(
        'publish_tf', default_value='true', description='Publish the ego transform using its header frame.',
    ))
    return LaunchDescription(arguments + [OpaqueFunction(function=launch_nodes)])
