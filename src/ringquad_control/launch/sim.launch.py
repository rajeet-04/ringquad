"""Gazebo + RViz + controller.

  ros2 topic pub --once /ringquad/mode std_msgs/msg/String "{data: fly}"
  ros2 topic pub --once /ringquad/goal geometry_msgs/msg/PoseStamped "{pose: {position: {x: 1.0, z: 1.5}}}"
  ros2 topic pub --once /ringquad/mode std_msgs/msg/String "{data: land}"
  ros2 topic pub --once /ringquad/mode std_msgs/msg/String "{data: drive}"
  ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ringquad_description'), 'launch', 'gazebo.launch.py')),
        launch_arguments={'rviz': LaunchConfiguration('rviz'), 'world': LaunchConfiguration('world'),
                          'tread': LaunchConfiguration('tread'), 'gui': LaunchConfiguration('gui'), 'steering': LaunchConfiguration('steering')}.items())

    return LaunchDescription([
        DeclareLaunchArgument('steering', default_value='fixed', choices=['fixed', 'front']),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('world', default_value='ringquad'),
        DeclareLaunchArgument('tread', default_value='rubber', description='rubber (spec) or omni'),
        gazebo,
        Node(package='ringquad_control', executable='controller',
             parameters=[{'use_sim_time': True, 'steering': LaunchConfiguration('steering')}], output='screen'),
    ])
