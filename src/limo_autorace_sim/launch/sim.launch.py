from pathlib import Path
import os
import tempfile
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share=Path(get_package_share_directory('limo_autorace_sim'))
    arg=lambda k: LaunchConfiguration(k).perform(context)
    mode=arg('mode')
    if mode not in ('ackermann','diff'): raise ValueError('mode must be ackermann or diff')
    vehicle=Path(arg('vehicle_config')); sensors=Path(arg('sensor_config'))
    v=yaml.safe_load(vehicle.read_text())
    iface=yaml.safe_load(Path(arg('interfaces')).read_text())
    description=xacro.process_file(str(share/'urdf/limo_sim.xacro'),mappings={
        'vehicle_config':str(vehicle),'sensor_config':str(sensors),'mode':mode}).toxml()
    temp=Path(tempfile.mkdtemp(prefix='limo_autorace_'))
    # Absolute paths make Gazebo independent of ROS package:// URI resolution.
    description=description.replace('package://limo_autorace_sim/','file://'+str(share)+'/')
    (temp/'limo.urdf').write_text(description)
    bridges=[]
    def bridge(ros,gz,ros_type,gz_type,direction='GZ_TO_ROS',qos=None):
        b=dict(ros_topic_name=ros,gz_topic_name=gz,ros_type_name=ros_type,gz_type_name=gz_type,direction=direction)
        if qos: b['qos_profile']=qos
        bridges.append(b)
    bridge('/clock','/clock','rosgraph_msgs/msg/Clock','gz.msgs.Clock',qos='CLOCK')
    bridge('/sim/cmd_vel','/sim/cmd_vel','geometry_msgs/msg/Twist','gz.msgs.Twist','ROS_TO_GZ')
    bridge(iface['odom'],'/sim/odom','nav_msgs/msg/Odometry','gz.msgs.Odometry')
    bridge('/tf','/sim/odom_tf','tf2_msgs/msg/TFMessage','gz.msgs.Pose_V')
    bridge('/joint_states','/sim/joint_states','sensor_msgs/msg/JointState','gz.msgs.Model')
    bridge('/evaluation/ground_truth','/evaluation/ground_truth','nav_msgs/msg/Odometry','gz.msgs.Odometry')
    for ros,gz,rt,gt in [
        ('image','camera/image','Image','Image'),('camera_info','camera/camera_info','CameraInfo','CameraInfo'),
        ('depth','depth/image','Image','Image'),('depth_info','depth/camera_info','CameraInfo','CameraInfo'),
        ('scan','scan','LaserScan','LaserScan'),('imu','imu','Imu','IMU')]:
        bridge('/sim/raw/'+ros,'/sim/'+gz,'sensor_msgs/msg/'+rt,'gz.msgs.'+gt,qos='SENSOR_DATA')
    (temp/'bridge.yaml').write_text(yaml.safe_dump(bridges))
    gz=['gz','sim','-r','-v','3',str(share/'worlds/autorace.sdf')]
    if arg('gui')=='false': gz += ['-s','--headless-rendering']
    else: gz += ['--gui-config',str(share/'config/gui.config')]
    actions=[
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',str(share/'models')+os.pathsep+os.environ.get('GZ_SIM_RESOURCE_PATH','')),
        ExecuteProcess(cmd=gz,output='screen'),
        Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'use_sim_time':True,'robot_description':description}],output='screen'),
        Node(package='ros_gz_bridge',executable='parameter_bridge',parameters=[{'config_file':str(temp/'bridge.yaml'),'use_sim_time':True}],output='screen'),
        Node(package='ros_gz_sim',executable='create',arguments=['-world','autorace','-file',str(temp/'limo.urdf'),'-name','limo','-x',arg('x'),'-y',arg('y'),'-z',str(v['wheel_radius']+.008),'-Y',arg('yaw')],output='screen'),
        Node(package='limo_autorace_sim',executable='command_guard',name='command_guard',parameters=[str(share/'config/runtime.yaml'),{'mode':mode,'wheelbase':v['wheelbase'],'central_steering_limit':v['central_steering_limit']}],remappings=[('cmd_vel',iface['cmd_vel'])],output='screen'),
        Node(package='limo_autorace_sim',executable='sensor_delay',name='sensor_delay',parameters=[str(share/'config/runtime.yaml')],remappings=[(k,iface[k]) for k in ('image','camera_info','depth','depth_info','scan','imu')],output='screen')]
    if arg('rviz')=='true':
        actions.append(Node(package='rviz2',executable='rviz2',arguments=['-d',str(share/'rviz/autorace.rviz')],parameters=[{'use_sim_time':True}]))
    return actions


def generate_launch_description():
    s=Path(get_package_share_directory('limo_autorace_sim'))
    defaults={'mode':'ackermann','gui':'true','rviz':'false','x':'1.6','y':'5.57','yaw':'0.0',
              'vehicle_config':str(s/'config/vehicle.yaml'),'sensor_config':str(s/'config/sensors.yaml'),
              'interfaces':str(s/'config/interfaces.yaml')}
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()]+[OpaqueFunction(function=setup)])
