"""Demo course: Gazebo (worlds/demo.sdf) + controller, then the mission once the sim is up.

  ros2 launch ringquad_control demo.launch.py            # runs the mission
  ros2 launch ringquad_control demo.launch.py mission:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ringquad_control'), 'launch', 'sim.launch.py')),
        launch_arguments={'world': 'demo', 'rviz': LaunchConfiguration('rviz'),
                          'tread': LaunchConfiguration('tread')}.items())

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('mission', default_value='true'),
        DeclareLaunchArgument('tread', default_value='rubber', description='rubber (spec) or omni'),
        sim,
        # the mission waits for odometry and the controller itself; the delay just lets Gazebo settle
        TimerAction(period=10.0, actions=[
            Node(package='ringquad_control', executable='mission', output='screen',
                 parameters=[{'use_sim_time': True}],
                 condition=IfCondition(LaunchConfiguration('mission'))),
        ]),
    ])
