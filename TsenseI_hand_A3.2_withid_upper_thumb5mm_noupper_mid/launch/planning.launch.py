from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    share=Path(get_package_share_directory('tsensei_hand_right_raised5mm_20_outputs_description'))
    description=(share/'urdf/TsenseI_hand_right_ros.urdf').read_text(encoding='utf-8')
    semantic=(share/'config/TsenseI_hand_right.srdf').read_text(encoding='utf-8')
    planning=yaml.safe_load((share/'config/ompl_planning.yaml').read_text(encoding='utf-8'))
    limits=yaml.safe_load((share/'config/moveit_joint_limits.yaml').read_text(encoding='utf-8'))
    parameters={'robot_description':description,'robot_description_semantic':semantic,'robot_description_planning':limits,'planning_pipelines':['ompl'],'default_planning_pipeline':'ompl','ompl':planning,'allow_trajectory_execution':False,'publish_robot_description_semantic':True,'planning_scene_monitor':{'publish_planning_scene':True,'publish_geometry_updates':True,'publish_state_updates':True,'publish_transforms_updates':True}}
    return LaunchDescription([
        DeclareLaunchArgument('gui',default_value='true'),
        Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':description}]),
        Node(package='joint_state_publisher',executable='joint_state_publisher',parameters=[{'robot_description':description}]),
        Node(package='tf2_ros',executable='static_transform_publisher',arguments=['--frame-id','world','--child-frame-id','base_link']),
        Node(package='moveit_ros_move_group',executable='move_group',parameters=[parameters]),
        Node(package='rviz2',executable='rviz2',arguments=['-d',str(share/'rviz/planning.rviz')],parameters=[parameters],condition=IfCondition(LaunchConfiguration('gui'))),
    ])
