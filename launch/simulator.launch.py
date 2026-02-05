from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from typing import List

def generate_launch_description():
    actuatorDelay_arg = DeclareLaunchArgument(
        '/simulator/actuator_delay', default_value=TextSubstitution(text='0.1')
    )

    perceptionErrorTime_arg = DeclareLaunchArgument(
        '/simulator/perception_error_time', default_value=TextSubstitution(text='10.0')
    )

    perceptionErrorLength_arg = DeclareLaunchArgument(
        '/simulator/perception_error_length', default_value=TextSubstitution(text='0.5')
    )

    objectDisappearTime_arg = DeclareLaunchArgument(
        '/simulator/object_disappear_time', default_value=TextSubstitution(text='12.0')
    )
    
    initialEgoState_arg = DeclareLaunchArgument(
        '/simulator/initial_ego_state', default_value=TextSubstitution(text='[0.0, 10.0, 0.0]'),
        description = 'Initital ego state as [x, vx, ax]'
    )

    initialObjectState_arg = DeclareLaunchArgument(
        '/simulator/initial_object_state', default_value=TextSubstitution(text='[100.0, 8.0, -5.0]'),
        description = 'Initital object state as [x, vx, ax]'
    )

    useInternalController_arg = DeclareLaunchArgument(
        '/simulator/use_internal_controller', default_value=TextSubstitution(text='true')
    )

    simulator = Node(
        package="simulator",
        executable="simulator",
        name="simulator",
        output="screen",
        parameters=[{
            '/simulator/actuator_delay': LaunchConfiguration('/simulator/actuator_delay'),
            '/simulator/perception_error_time': LaunchConfiguration('/simulator/perception_error_time'),
            '/simulator/perception_error_length': LaunchConfiguration('/simulator/perception_error_length'),
            '/simulator/object_disappear_time': LaunchConfiguration('/simulator/object_disappear_time'),
            '/simulator/initial_ego_state': ParameterValue(
                LaunchConfiguration('/simulator/initial_ego_state'),
                value_type = List[float]
            ),
            '/simulator/initial_object_state': ParameterValue(
                LaunchConfiguration('/simulator/initial_object_state'),
                value_type = List[float]
            ),
            '/simulator/use_internal_controller': LaunchConfiguration('/simulator/use_internal_controller'),
        }]
    )

    tf_broadcaster = Node(
        package="simulator",
        executable="tf_broadcaster",
        name="tf_broadcaster",
        output="screen"
    )

    return LaunchDescription([
        actuatorDelay_arg,
        perceptionErrorTime_arg,
        perceptionErrorLength_arg,
        objectDisappearTime_arg,
        initialEgoState_arg,
        initialObjectState_arg,
        useInternalController_arg,
        simulator,
        tf_broadcaster
    ])
