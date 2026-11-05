"""Publish the adopted zero or a named reference pose."""
from pathlib import Path
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    share = Path(get_package_share_directory('hand_with_tip_shells_without_screw_zero_description'))
    return LaunchDescription([
        DeclareLaunchArgument('pose', default_value=str(share/'examples/zero.json')),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': (share/'urdf/hand_1009_ros.urdf').read_text(encoding='utf-8')}]),
        ExecuteProcess(cmd=[sys.executable, str(share/'tools/publish_pose.py'), '--pose', LaunchConfiguration('pose')], output='screen')])
