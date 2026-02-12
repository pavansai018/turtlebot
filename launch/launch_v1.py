import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    urdf = os.path.join(get_package_share_directory(
        'turtlebot'), 'urdf', 'turtlebot3_burger.urdf')

    robot_desc = ParameterValue(Command(
        [
            'xacro ', 
            urdf, 
            ' ', 
            # 'ros2_control_yaml:=', control_yaml_file
            ]
        ),value_type=str)

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    # world_file = os.path.join(get_package_share_directory('spiderbytes'), 'world', 'black_circle.sdf')

    # Include the gz sim launch file  
    gz_sim_share = get_package_share_directory("ros_gz_sim")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args" :  '-r empty.sdf' #f'-r {world_file}' #'-r empty.sdf'
        }.items()
    )
    # Spawn Rover Robot
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "/robot_description",
            "-name", "turtlebot_burger",
            "-allow_renaming", "true",
            "-z", "0.5",
        ]
    )
    
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
            # '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
        ],
    )

    # Robot state publisher
    params = {'use_sim_time': use_sim_time, 'robot_description': robot_desc}
    start_robot_state_publisher_cmd = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[params],
            arguments=[])
    # joint_state_publisher_gui: publishes /joint_states with sliders
    jsp_gui_params = {
        'use_sim_time': use_sim_time,
        # 'robot_description': robot_desc
    }
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'
            ])

        ]),
        launch_arguments={
            'slam_params_file': PathJoinSubstitution(
                [
                    FindPackageShare('turtlebot'), 'config', 'mapper_params_online_async.yaml',
                ]
            ),
            'use_sim_time': 'true',

        }.items()
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    # Launch Gazebo
    ld.add_action(gz_sim)
    ld.add_action(gz_spawn_entity)
    ld.add_action(gz_ros2_bridge)
    # Launch Robot State Publisher
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(slam_toolbox)
    return ld