import os
import shutil
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_rover_desc = FindPackageShare('rover_description')
    pkg_rover_gazebo = FindPackageShare('rover_gazebo')
    pkg_tof_sim = FindPackageShare('tof_sensor_sim')
    pkg_wifi_sim = FindPackageShare('wifi_rf_sim')
    pkg_sensor_fusion = FindPackageShare('sensor_fusion')

    # Force wall clock for 100% reliable 30 FPS rendering in RViz
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    xacro_file = PathJoinSubstitution([pkg_rover_desc, 'urdf', 'rover.urdf.xacro'])
    robot_description_content = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'robot_description': robot_description_content
        }]
    )

    world_file = PathJoinSubstitution([pkg_rover_gazebo, 'worlds', 'rf_arena.world'])

    if shutil.which('ign'):
        sim_cmd = ['ign', 'gazebo', '-s', '-r', world_file]
    elif shutil.which('gz'):
        sim_cmd = ['gz', 'sim', '-s', '-r', world_file]
    else:
        sim_cmd = ['gzserver', '--verbose', '-s', 'libgazebo_ros_factory.so', world_file]

    gz_sim_process = ExecuteProcess(
        cmd=sim_cmd,
        output='screen'
    )

    gz_bridge_node = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
             '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
             '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
             '/model/multisensor_fusion_rover/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
             '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
             '/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU'],
        output='screen'
    )

    tof_sim_node = Node(
        package='tof_sensor_sim',
        executable='tof_publisher_node',
        name='tof_publisher_node',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    wifi_sim_node = Node(
        package='wifi_rf_sim',
        executable='wifi_rssi_sim_node',
        name='wifi_rssi_sim_node',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    ekf_fusion_node = Node(
        package='sensor_fusion',
        executable='ekf_fusion_node',
        name='ekf_fusion_node',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    rviz_config_file = PathJoinSubstitution([pkg_rover_gazebo, 'config', 'rf_slam.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use wall clock'),
        robot_state_pub_node,
        gz_sim_process,
        gz_bridge_node,
        tof_sim_node,
        wifi_sim_node,
        ekf_fusion_node,
        rviz_node
    ])
