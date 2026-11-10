from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


PARAMETERS = {
    'scenario_file': (PathJoinSubstitution([
        FindPackageShare('scenario_generation'), 'scenario_files',
        'occluded_pedestrian_costmap_data.mat']), str),
    'trim_zero_padding': ('true', bool),
    'scenario_distance': ('75.0', float),
    'scenario_lookahead_time': ('6.0', float),
    'object_distance': ('150.0', float),
    'lane_width': ('3.85', float),
    'target_path_width': ('3.85', float),
    'crossing_width': ('3.0', float),
    'prediction_horizon': ('4.0', float),
    'enable_occlusion': ('true', bool),
    'generate_virtual_objects': ('true', bool),
    'crossing_pedestrian_probability': ('0.5', float),
    'roadside_pedestrian_probability': ('0.02', float),
    'virtual_vehicle_probability': ('0.5', float),
    'pedestrian_walking_speed': ('0.8', float),
    'pedestrian_running_speed': ('3.0', float),
    'pedestrian_walking_confidence': ('0.9', float),
    'virtual_vehicle_fast_speed': ('15.0', float),
    'virtual_vehicle_slow_speed': ('8.0', float),
    'virtual_vehicle_fast_confidence': ('0.7', float),
    'maximum_virtual_objects': ('64', int),
    'initial_x': ('auto', float),
    'initial_y': ('auto', float),
    'initial_yaw': ('auto', float),
    'initial_displacement': ('0.0', float),
    'publish_ego_tf': ('true', bool),
}


def launch_node(context):
    parameters = {}
    for name, (_, value_type) in PARAMETERS.items():
        if name in ('initial_x', 'initial_y', 'initial_yaw'):
            if LaunchConfiguration(name).perform(context).strip().lower() == 'auto':
                continue
        parameters[name] = ParameterValue(LaunchConfiguration(name), value_type=value_type)
    return [Node(
        package='scenario_generation', executable='scenario_generator', name='scenario_generator',
        output='screen', parameters=[parameters],
    )]


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(name, default_value=default)
        for name, (default, _) in PARAMETERS.items()
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=launch_node)])