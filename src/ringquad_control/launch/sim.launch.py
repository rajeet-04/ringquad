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
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ringquad_description'), 'launch', 'gazebo.launch.py')),
        launch_arguments={'rviz': LaunchConfiguration('rviz'), 'world': LaunchConfiguration('world'),
                          'tread': LaunchConfiguration('tread'), 'gui': LaunchConfiguration('gui'), 'steering': LaunchConfiguration('steering'), 'tread_collision': LaunchConfiguration('tread_collision'), 'self_collision': LaunchConfiguration('self_collision'), 'servo_torque': LaunchConfiguration('servo_torque'), 'servo_speed': LaunchConfiguration('servo_speed'), 'friction_scale': LaunchConfiguration('friction_scale'), 'knuckle_mass': LaunchConfiguration('knuckle_mass'), 'ring_offset': LaunchConfiguration('ring_offset'), 'steering_limit_deg': LaunchConfiguration('steering_limit_deg'), 'fold_ground_deg': LaunchConfiguration('fold_ground_deg'), 'drivetrain': LaunchConfiguration('drivetrain'), 'lock_break_torque': LaunchConfiguration('lock_break_torque')}.items())

    return LaunchDescription([
        DeclareLaunchArgument('tread_collision', default_value='crowned', choices=['rounded','cylinder','crowned']),
        DeclareLaunchArgument('self_collision', default_value='true'),
        DeclareLaunchArgument('fold_ground_deg', default_value='90'),
        DeclareLaunchArgument('drivetrain', default_value='separate', choices=['separate','shared']),
        DeclareLaunchArgument('lock_break_torque', default_value='8.0'),
        DeclareLaunchArgument('servo_torque', default_value='1.0'),
        DeclareLaunchArgument('servo_speed', default_value='1.5'),
        DeclareLaunchArgument('friction_scale', default_value='1.0'),
        DeclareLaunchArgument('knuckle_mass', default_value='0.04'),
        DeclareLaunchArgument('ring_offset', default_value='0.1'),
        DeclareLaunchArgument('steering_limit_deg', default_value='25'),
        DeclareLaunchArgument('steering', default_value='front', choices=['fixed', 'front']),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('world', default_value='ringquad'),
        DeclareLaunchArgument('tread', default_value='rubber', description='rubber (spec) or omni'),
        gazebo,
        Node(package='ringquad_control', executable='controller',
             parameters=[{'use_sim_time': True, 'drivetrain': LaunchConfiguration('drivetrain'), 'steering': LaunchConfiguration('steering'),
                          'fold_ground_deg': ParameterValue(LaunchConfiguration('fold_ground_deg'), value_type=float),
                          'knuckle_mass': ParameterValue(LaunchConfiguration('knuckle_mass'), value_type=float),
                          'steering_limit_deg': ParameterValue(LaunchConfiguration('steering_limit_deg'), value_type=float),
                          'ring_offset': ParameterValue(LaunchConfiguration('ring_offset'), value_type=float), 'tread_collision': LaunchConfiguration('tread_collision')}], output='screen'),
    ])
