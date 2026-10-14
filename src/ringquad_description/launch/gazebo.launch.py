"""Gazebo Harmonic: spawn the robot, bridge topics, show it in RViz.

Fold all arms:   ros2 topic pub --once /ringquad/fold std_msgs/msg/Float64 "{data: 1.396}"
Unfold:          ros2 topic pub --once /ringquad/fold std_msgs/msg/Float64 "{data: 0.0}"
Spin one ring:   ros2 topic pub --once /ringquad/ring_fl std_msgs/msg/Float64 "{data: 10.0}"
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('ringquad_description')
    robot_description = xacro.process_file(os.path.join(share, 'urdf', 'ringquad.urdf.xacro')).toxml()

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r empty.sdf'}.items())

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(share)),
        gz_sim,
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description, 'use_sim_time': True}]),
        Node(package='ros_gz_sim', executable='create',
             arguments=['-topic', 'robot_description', '-name', 'ringquad', '-z', '0.3']),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             parameters=[{'config_file': os.path.join(share, 'config', 'bridge.yaml'),
                          'use_sim_time': True}]),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(share, 'rviz', 'ringquad.rviz')],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
