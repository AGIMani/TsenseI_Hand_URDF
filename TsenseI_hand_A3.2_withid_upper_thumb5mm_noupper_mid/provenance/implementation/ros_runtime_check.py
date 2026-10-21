"""Run in the existing Drake-Ubuntu ROS2 Jazzy runtime; workspace-local install."""
from pathlib import Path
import os, json, subprocess, time, signal, argparse, hashlib
ROOT=Path('/mnt/e/codex_work/导出urdf')
WORK=ROOT/'work/urdf_raised_5mm_20261011/right_hand_r03_20261013/ros_runtime'
PKG='tsensei_hand_right_raised5mm_20_outputs_description'
OUT=ROOT/'outputs/urdf_raised_5mm_20261011/TsenseI_右手_20关节_完整模型_20261013_r03'/PKG

def main():
    WORK.mkdir(parents=True,exist_ok=True)
    command=['colcon','--log-base',str(WORK/'log'),'build','--base-paths',str(OUT),'--build-base',str(WORK/'build'),'--install-base',str(WORK/'install'),'--packages-select',PKG,'--event-handlers','console_direct+']
    build=subprocess.run(command,cwd=WORK,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
    (WORK/'build.log').write_text(build.stdout,encoding='utf-8');print('ROS2 package build return code:',build.returncode,flush=True)
    assert build.returncode==0,'ROS2 package build failed'
    install=WORK/'install'/PKG;env=os.environ.copy();env['AMENT_PREFIX_PATH']=str(install)+':'+env.get('AMENT_PREFIX_PATH','');env['CMAKE_PREFIX_PATH']=str(install)+':'+env.get('CMAKE_PREFIX_PATH','');env['ROS_DOMAIN_ID']='83'
    os.environ.update(env)
    # Native ROS xacro expansion independently of the Windows Python package.
    import xacro, xml.etree.ElementTree as ET
    expanded=ET.fromstring(xacro.process_file(str(install/'share'/PKG/'urdf/TsenseI_hand_right_mock.urdf.xacro')).toxml())
    assert len(expanded.findall('joint'))==21 and len(expanded.findall('ros2_control/joint'))==20
    from ament_index_python.packages import get_package_prefix
    requested=['robot_state_publisher','joint_state_publisher','joint_state_publisher_gui','rviz2','controller_manager','joint_state_broadcaster','joint_trajectory_controller','moveit_ros_move_group','moveit_planners_ompl']
    available={}
    for p in requested:
        try:available[p]=get_package_prefix(p)
        except LookupError:available[p]=None
    import rclpy
    from sensor_msgs.msg import JointState
    from tf2_msgs.msg import TFMessage
    from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
    rclpy.init();node=rclpy.create_node('tsensei_r03_runtime_audit');states=[];transforms=set();static=set()
    node.create_subscription(JointState,'/joint_states',lambda msg:states.append({'names':list(msg.name),'positions':list(msg.position)}),10)
    node.create_subscription(TFMessage,'/tf',lambda msg:transforms.update(t.child_frame_id for t in msg.transforms),10)
    qos=QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(TFMessage,'/tf_static',lambda msg:static.update(t.child_frame_id for t in msg.transforms),qos)
    with (WORK/'display.log').open('w',encoding='utf-8') as log:
        process=subprocess.Popen(['ros2','launch',PKG,'display.launch.py','gui:=false','rviz:=false'],cwd=WORK,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            deadline=time.monotonic()+35
            while time.monotonic()<deadline and not(states and len(transforms)>=20 and static):
                rclpy.spin_once(node,timeout_sec=.2)
                assert process.poll() is None,'ROS display launch exited prematurely'
            assert states and len(states[-1]['names'])==20 and len(transforms)==20 and static=={'Right_hand_palm_link'},(states,transforms,static)
        finally:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=10)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
    node.destroy_node();rclpy.shutdown()
    digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    installed_ros=install/'share'/PKG/'urdf/TsenseI_hand_right_ros.urdf'
    assert digest(installed_ros)==digest(OUT/'urdf/TsenseI_hand_right_ros.urdf')
    result={'ROS_distribution':'jazzy','host':'existing Drake-Ubuntu WSL distribution','package_build_passed':True,'symlink_install_used':False,'installed_ROS_URDF_SHA256_matches_current_package':True,'ROS_URDF_SHA256':digest(installed_ros),'native_xacro_20_control_expansion_passed':True,'headless_display_launch_passed':True,'joint_state_name_count':len(states[-1]['names']),'joint_states':states[-1],'moving_TF_children':sorted(transforms),'fixed_TF_children':sorted(static),'installed_dependencies':available,'GUI_RViz_checked':False,'mock_control_runtime_checked':False,'planning_runtime_checked':False,'mock_planning_limitation':'Runtime dependencies missing where installed_dependencies is null; corresponding configuration is structurally validated and requires rosdep installation on target ROS2 host.'}
    (OUT/'checks/ROS2_runtime_validation.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False),flush=True)

if __name__=='__main__':main()
