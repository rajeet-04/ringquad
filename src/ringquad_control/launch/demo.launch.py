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
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ringquad_control'), 'launch', 'sim.launch.py')),
        launch_arguments={'world': 'demo', 'rviz': LaunchConfiguration('rviz'),
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
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('mission', default_value='true'),
        DeclareLaunchArgument('tread', default_value='rubber', description='omni (demo), rubber, or mixed'),
        sim,
        # the mission waits for odometry and the controller itself; the delay just lets Gazebo settle
        TimerAction(period=10.0, actions=[
            Node(package='ringquad_control', executable='mission', output='screen',
                 parameters=[{'use_sim_time': True, 'steering': LaunchConfiguration('steering'), 'fold_ground_deg': ParameterValue(LaunchConfiguration('fold_ground_deg'), value_type=float), 'ring_offset': ParameterValue(LaunchConfiguration('ring_offset'), value_type=float)}],
                 condition=IfCondition(LaunchConfiguration('mission'))),
        ]),
    ])
