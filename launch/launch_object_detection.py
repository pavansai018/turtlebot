import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    rate_limit_arg = DeclareLaunchArgument(
        'rate_limit',
        default_value='30.0',
    )
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw',
    )
    image_type_arg = DeclareLaunchArgument(
        'image_type',
        default_value='raw',
    )
    enable_ocr_arg = DeclareLaunchArgument(
        'enable_ocr',
        default_value='false',
    )
    
    object_detection_node = Node(
        package='turtlebot',
        executable='object_detection',
        name='YOLO_Object_Detection',
        output='screen',
        parameters=[
            {
                'rate_limit': LaunchConfiguration('rate_limit'),
                'image_topic': LaunchConfiguration('image_topic'),
                'image_type': LaunchConfiguration('image_type'),
                'enable_ocr': LaunchConfiguration('enable_ocr'),
            },
        ],
    )

    gps_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gps_tf_publisher',
        arguments=[
            '--x', '0.1',
            '--y', '-0.1',
            '--z', '0.5',
            '--roll', '3.14',
            '--pitch', '0.0',
            '--yaw', '-1.57',
            '--frame-id', 'base_link',
            '--child-frame-id', 'gps',
        ],
    )
    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(rate_limit_arg)
    ld.add_action(image_topic_arg)
    ld.add_action(image_type_arg)
    ld.add_action(enable_ocr_arg)
    ld.add_action(object_detection_node)
    ld.add_action(gps_publisher)
    return ld