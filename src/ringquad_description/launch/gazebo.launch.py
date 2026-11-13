"""Gazebo Harmonic: spawn the robot, bridge topics, optionally show RViz.

Use ringquad_control's sim.launch.py to also start the controller.
Raw fold command (no controller):
  ros2 topic pub --once /ringquad/fold std_msgs/msg/Float64 "{data: 1.396}"
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('ringquad_description')
    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(share, 'urdf', 'ringquad.urdf.xacro'), ' tread:=', LaunchConfiguration('tread'), ' steering:=', LaunchConfiguration('steering'), ' tread_collision:=', LaunchConfiguration('tread_collision'), ' self_collision:=', LaunchConfiguration('self_collision'), ' servo_torque:=', LaunchConfiguration('servo_torque'), ' servo_speed:=', LaunchConfiguration('servo_speed'), ' friction_scale:=', LaunchConfiguration('friction_scale'), ' knuckle_mass:=', LaunchConfiguration('knuckle_mass'), ' ring_offset:=', LaunchConfiguration('ring_offset'), ' steering_limit_deg:=', LaunchConfiguration('steering_limit_deg'), ' fold_ground_deg:=', LaunchConfiguration('fold_ground_deg'), ' drivetrain:=', LaunchConfiguration('drivetrain')]),
        value_type=str)
    world = PathJoinSubstitution([share, 'worlds', [LaunchConfiguration('world'), '.sdf']])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [PythonExpression(["'-r -s ' if '", LaunchConfiguration('gui'), "' == 'false' else '-r '"]), world]}.items())

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('tread_collision', default_value='crowned', choices=['rounded','cylinder','crowned']),
        DeclareLaunchArgument('self_collision', default_value='true'),
        DeclareLaunchArgument('fold_ground_deg', default_value='90'),
        DeclareLaunchArgument('drivetrain', default_value='separate', choices=['separate','shared']),
        DeclareLaunchArgument('servo_torque', default_value='1.0'),
        DeclareLaunchArgument('servo_speed', default_value='1.5'),
        DeclareLaunchArgument('friction_scale', default_value='1.0'),
        DeclareLaunchArgument('knuckle_mass', default_value='0.02'),
        DeclareLaunchArgument('ring_offset', default_value='0.1'),
        DeclareLaunchArgument('steering_limit_deg', default_value='25'),
        DeclareLaunchArgument('steering', default_value='front', choices=['fixed', 'front']),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('world', default_value='ringquad', description='ringquad (empty) or demo'),
        DeclareLaunchArgument('tread', default_value='rubber', description='rubber (spec) or omni'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(share)),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', os.path.join(share, '..', '..', 'lib') + os.pathsep + os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')),
        gz_sim,
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description, 'use_sim_time': True}]),
        Node(package='ros_gz_sim', executable='create',
             arguments=['-topic', 'robot_description', '-name', 'ringquad', '-z', '0.1']),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             parameters=[{'config_file': os.path.join(share, 'config', 'bridge.yaml'),
                          'use_sim_time': True}]),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(share, 'rviz', 'ringquad.rviz')],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
