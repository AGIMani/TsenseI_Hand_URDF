"""Publish only the twenty reduced output joints, using standard ROS 2 nodes."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    root=Path(get_package_share_directory('hand_1009_reduced_description'))
    xml=(root/'urdf/hand_1009_20j_ros.urdf').read_text(encoding='utf-8')
    description={'robot_description':xml, 'use_sim_time':ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('gui',default_value='true'),
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[description]),
        Node(package='joint_state_publisher_gui',executable='joint_state_publisher_gui',parameters=[description],condition=IfCondition(LaunchConfiguration('gui'))),
        Node(package='joint_state_publisher',executable='joint_state_publisher',parameters=[description],condition=UnlessCondition(LaunchConfiguration('gui'))),
    ])
