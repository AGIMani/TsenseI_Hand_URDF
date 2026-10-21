from pathlib import Path
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    share=Path(get_package_share_directory('tsensei_hand_right_raised5mm_20_outputs_description'))
    description=xacro.process_file(str(share/'urdf/TsenseI_hand_right_mock.urdf.xacro')).toxml()
    return LaunchDescription([
        DeclareLaunchArgument('rviz',default_value='true'),
        Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':description}]),
        Node(package='controller_manager',executable='ros2_control_node',parameters=[str(share/'config/ros2_controllers_mock.yaml')]),
        Node(package='controller_manager',executable='spawner',arguments=['joint_state_broadcaster','hand_controller','--controller-manager','/controller_manager','--controller-manager-timeout','60','--param-file',str(share/'config/ros2_controllers_mock.yaml')]),
        Node(package='rviz2',executable='rviz2',arguments=['-d',str(share/'rviz/hand.rviz')],condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
