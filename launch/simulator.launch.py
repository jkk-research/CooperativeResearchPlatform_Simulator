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
            FindPackageShare('simulator'), 'scenario_files',
            'roundabout_with_traffic_and_pedestrian_costmap_data.mat',
        ]), str, 'MAT scenario file; qg coordinates are interpreted as map coordinates.'),
    'simulation_step': ('0.05', float, 'Ego integration step and publication period in simulation seconds.'),
    'real_time_factor': ('1.0', float, 'Simulation seconds per wall-clock second.'),
    'stop_at_scenario_end': ('true', bool, 'Freeze simulation and stop publication at the final sample.'),
    'trim_zero_padding': ('true', bool, 'Remove trailing zero-position/zero-speed padding from moving actors.'),
    'scenario_distance': ('75.0', float, 'Radius and along-corridor distance of published road geometry in meters.'),
    'object_distance': ('150.0', float, 'Maximum published object distance from ego in meters.'),
    'lane_width': ('3.85', float, 'Full width of each drivable-surface corridor in meters.'),
    'target_path_width': ('3.85', float, 'Full width of the target-path corridor in meters.'),
    'crossing_width': ('3.0', float, 'Pedestrian-crossing width in meters when absent from the MAT file.'),
    'prediction_horizon': ('4.0', float, 'Maximum object prediction horizon in seconds.'),
    'initial_x': ('auto', float, 'Initial map x in meters, or auto for the first target waypoint.'),
    'initial_y': ('auto', float, 'Initial map y in meters, or auto for the first target waypoint.'),
    'initial_yaw': ('auto', float, 'Initial map yaw in radians, or auto for the target-path tangent.'),
    'initial_speed': ('0.0', float, 'Initial ego forward speed in m/s.'),
    'initial_acceleration': ('0.0', float, 'Initial ego longitudinal acceleration in m/s^2.'),
    'use_internal_controller': ('true', bool, 'Follow targetPath_qg internally; otherwise use commanded steering.'),
    'use_scenario_speed_when_uncommanded': ('true', bool, 'Use speedLimit until the first external control command.'),
    'command_timeout': ('1.0', float, 'Wall-clock timeout after a command, then target zero speed.'),
    'wheelbase': ('2.7', float, 'Kinematic bicycle wheelbase in meters.'),
    'maximum_acceleration': ('9.81', float, 'Positive acceleration limit in m/s^2.'),
    'maximum_deceleration': ('9.81', float, 'Positive braking magnitude limit in m/s^2.'),
    'maximum_steering': ('0.6', float, 'Absolute front tire steering limit in radians; less than pi/2.'),
    'speed_gain': ('6.0', float, 'Proportional speed-controller gain in 1/s.'),
    'lookahead_time': ('1.5', float, 'Internal lateral-controller lookahead time in seconds.'),
    'minimum_lookahead': ('2.0', float, 'Minimum internal lateral-controller lookahead distance in meters.'),
}


def launch_nodes(context):
    parameters = {}
    for name, (_, value_type, _) in PARAMETERS.items():
        if name in ('initial_x', 'initial_y', 'initial_yaw'):
            if LaunchConfiguration(name).perform(context).strip().lower() == 'auto':
                continue
        parameters[name] = ParameterValue(LaunchConfiguration(name), value_type=value_type)
    return [
        Node(
            package='simulator', executable='simulator', name='simulator',
            output='screen', parameters=[parameters],
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
        'publish_tf', default_value='true', description='Publish the map-to-base_link transform from ego.',
    ))
    return LaunchDescription(arguments + [OpaqueFunction(function=launch_nodes)])
