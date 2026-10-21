from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    share=Path(get_package_share_directory('tsensei_hand_right_raised5mm_20_outputs_description'))
    description=(share/'urdf/TsenseI_hand_right_ros.urdf').read_text(encoding='utf-8')
    return LaunchDescription([
        DeclareLaunchArgument('gui',default_value='true'),
        DeclareLaunchArgument('rviz',default_value='true'),
        Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':description}]),
        Node(package='joint_state_publisher_gui',executable='joint_state_publisher_gui',parameters=[{'robot_description':description}],condition=IfCondition(LaunchConfiguration('gui'))),
        Node(package='joint_state_publisher',executable='joint_state_publisher',parameters=[{'robot_description':description}],condition=UnlessCondition(LaunchConfiguration('gui'))),
        Node(package='rviz2',executable='rviz2',arguments=['-d',str(share/'rviz/hand.rviz')],condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
