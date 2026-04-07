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
    detection_model_arg = DeclareLaunchArgument(
        'detection_model',
        default_value='yolov8l.pt',
    )
    visualize_detection_arg = DeclareLaunchArgument(
        'visualize_detection',
        default_value='true',
    )
    visualize_segmentation_arg = DeclareLaunchArgument(
        'visualize_segmentation',
        default_value='false',
    )
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='true',
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
                'detection_model': LaunchConfiguration('detection_model'),
                'visualize_detection': LaunchConfiguration('visualize_detection'),
                'visualize_segmentation': LaunchConfiguration('visualize_segmentation'),
                'debug': LaunchConfiguration('debug'),
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

    # camera_publisher = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='camera_tf_publisher',
    #     arguments=[
    #         '--x', '0.13',
    #         '--y', '0.0',
    #         '--z', '0.0',
    #         '--roll', '0.0',
    #         '--pitch', '0.0',
    #         '--yaw', '0.0',
    #         '--frame-id', 'base_scan',
    #         '--child-frame-id', 'camera',
    #     ],
    # )
    camera_mount_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_mount_tf_publisher',
        arguments=[
            '--x', '0.13',
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_scan',
            '--child-frame-id', 'camera_mount',
        ],
    )

    camera_optical_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_optical_tf_publisher',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '-1.57079632679',
            '--pitch', '0.0',
            '--yaw', '-1.57079632679',
            '--frame-id', 'camera_mount',
            '--child-frame-id', 'camera',
        ],
    )
    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(rate_limit_arg)
    ld.add_action(visualize_detection_arg)
    ld.add_action(visualize_segmentation_arg)
    ld.add_action(detection_model_arg)
    ld.add_action(image_topic_arg)
    ld.add_action(image_type_arg)
    ld.add_action(enable_ocr_arg)
    ld.add_action(debug_arg)
    ld.add_action(object_detection_node)
    ld.add_action(gps_publisher)
    ld.add_action(camera_mount_publisher)
    ld.add_action(camera_optical_publisher)
    # ld.add_action(camera_publisher)
    return ld