"""Physical integration test. Ground truth is confined to this evaluation node."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import time
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo, JointState, Imu
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage


def stamp(msg):
    t=msg.header.stamp
    return t.sec+t.nanosec*1e-9


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


class Probe(Node):
    def __init__(self,out,mode='ackermann'):
        super().__init__('autorace_probe',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
        self.out=out; out.mkdir(parents=True,exist_ok=True)
        self.mode=mode
        self.counts=Counter(); self.last={}; self.trace=[]; self.transforms=set(); self.joints={}
        self.image_stamps=set(); self.info_stamps=set(); self.images=[]
        self.phase='startup'; self.clock=0.0; self.backwards=False
        self.joint_trace=[]
        self.pub=self.create_publisher(Twist,'/cmd_vel',10)
        for typ,topic,key in [(Odometry,'/evaluation/ground_truth','truth'),(Odometry,'/odom','odom'),
            (Image,'/camera/color/image_raw','image'),(CameraInfo,'/camera/color/camera_info','info'),
            (JointState,'/joint_states','joints'),(Imu,'/imu','imu')]:
            self.create_subscription(typ,topic,lambda m,k=key:self.receive(k,m),qos_profile_sensor_data)
        self.create_subscription(Clock,'/clock',self.on_clock,qos_profile_sensor_data)
        self.create_subscription(TFMessage,'/tf',self.on_tf,100)
        self.create_subscription(TFMessage,'/tf_static',self.on_tf,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def on_tf(self,msg):
        for t in msg.transforms: self.transforms.add((t.header.frame_id,t.child_frame_id))

    def on_clock(self,msg):
        t=msg.clock.sec+msg.clock.nanosec*1e-9
        if t<self.clock: self.backwards=True
        self.clock=t

    def receive(self,key,msg):
        self.counts[key]+=1; self.last[key]=msg
        if key=='truth':
            p=msg.pose.pose.position; q=msg.pose.pose.orientation
            self.trace.append(dict(t=stamp(msg),phase=self.phase,x=p.x,y=p.y,z=p.z,yaw=yaw(q)))
        if key=='joints':
            for n,p in zip(msg.name,msg.position): self.joints.setdefault(n,[]).append(p)
            self.joint_trace.append(dict(phase=self.phase, positions=dict(zip(msg.name,msg.position))))
        if key in ('image','info'):
            (self.image_stamps if key=='image' else self.info_stamps).add(round(stamp(msg),6))
        if key=='image' and self.phase not in self.images:
            cv2.imwrite(str(self.out/(self.phase+'.png')),CvBridge().imgmsg_to_cv2(msg,'bgr8'))
            self.images.append(self.phase)

    def run_phase(self,name,duration,v=0.,w=0.,send=True):
        self.phase=name; start=self.clock; wall=time.monotonic(); last_sent=-math.inf
        while self.clock-start < duration:
            if time.monotonic()-wall > 120: raise TimeoutError('Simulation did not advance fast enough')
            if send and self.clock-last_sent >= .02:
                cmd=Twist();cmd.linear.x=float(v);cmd.angular.z=float(w);self.pub.publish(cmd)
                last_sent=self.clock
            rclpy.spin_once(self,timeout_sec=.02)

    def result(self):
        phases={p:[x for x in self.trace if x['phase']==p] for p in {x['phase'] for x in self.trace}}
        def distance(p):
            a,b=phases[p][0],phases[p][-1];return math.hypot(b['x']-a['x'],b['y']-a['y'])
        def turn(p):
            return math.atan2(math.sin(phases[p][-1]['yaw']-phases[p][0]['yaw']),math.cos(phases[p][-1]['yaw']-phases[p][0]['yaw']))
        checks={}
        def check(k,v):checks[k]=bool(v)
        check('required_topics',all(self.counts[k]>5 for k in ['truth','odom','image','info','joints','imu']))
        check('clock_monotonic',self.clock>1 and not self.backwards)
        steady=[x for x in self.trace if x['phase']!='startup']
        check('contact_height',steady and min(x['z'] for x in steady)>0.025 and max(x['z'] for x in steady)<0.075)
        check('forward_motion',distance('straight')>0.4)
        check('left_yaw_positive',turn('left')>0.12)
        check('right_yaw_negative',turn('right')<-.12)
        stop=phases['stop']
        check('stops',math.hypot(stop[-1]['x']-stop[-10]['x'],stop[-1]['y']-stop[-10]['y'])<.02)
        timeout=phases['timeout']
        check('command_timeout_stops',math.hypot(timeout[-1]['x']-timeout[-10]['x'],timeout[-1]['y']-timeout[-10]['y'])<.02)
        check('optical_frame',self.last['image'].header.frame_id=='camera_optical_frame' and self.last['info'].header.frame_id=='camera_optical_frame')
        check('camera_info_dimensions',self.last['image'].width==self.last['info'].width and self.last['image'].height==self.last['info'].height)
        info=self.last['info']; expected=info.width/(2*math.tan(1.2/2))
        check('camera_intrinsics',abs(info.k[0]-expected)<1 and abs(info.k[4]-expected)<1)
        check('matched_image_info_timestamps',len(self.image_stamps & self.info_stamps)>10)
        check('tf_connected',('odom','base_link') in self.transforms and ('camera_link','camera_optical_frame') in self.transforms)
        check('front_wheels_rotate',all(max(self.joints[k])-min(self.joints[k])>1 for k in ['front_left_wheel','front_right_wheel']))
        if self.mode == 'ackermann':
            check('front_wheels_steer_both_ways',all(min(self.joints.get(k,[0]))<-.1 and max(self.joints.get(k,[0]))>.1 for k in ['front_left_steering','front_right_steering']))
        else:
            check('no_steering_joints',not any('steering' in k for k in self.joints))
            # Four-wheel skid steering rotates about the chassis centre, while
            # ground truth uses the rear axle: bound drift of the chassis centre.
            for phase,sign in [('spin_left',1),('spin_right',-1)]:
                check(phase+'_yaw',sign*turn(phase)>.25)
                a,b=phases[phase][0],phases[phase][-1]
                centre=lambda p:(p['x']+.1*math.cos(p['yaw']),p['y']+.1*math.sin(p['yaw']))
                check(phase+'_centre_drift',math.dist(centre(a),centre(b))<.08)
                joints=[j['positions'] for j in self.joint_trace if j['phase']==phase]
                check(phase+'_opposite_wheels',all(
                    sign*(joints[-1][k]-joints[0][k])*(-1 if 'left' in k else 1)>.2
                    for k in ('front_left_wheel','rear_left_wheel','front_right_wheel','rear_right_wheel')))
            tail=phases['spin_timeout'][-10:]
            check('spin_timeout_stops_rotation',abs(tail[-1]['yaw']-tail[0]['yaw'])<.02)
            check('spin_timeout_stops_translation',math.hypot(tail[-1]['x']-tail[0]['x'],tail[-1]['y']-tail[0]['y'])<.02)
        r=dict(mode=self.mode,checks=checks,counts=dict(self.counts),tf=sorted(self.transforms),
               travel_m=distance('straight'),left_yaw_rad=turn('left'),right_yaw_rad=turn('right'),
               z_range_m=[min(x['z'] for x in steady),max(x['z'] for x in steady)],
               camera_frame=self.last['image'].header.frame_id,clock_end=self.clock,
               limitation='Nominal CAD and source-model geometry, not measured real Pro; visual lane visibility requires image review')
        if self.mode == 'diff':
            r['spin_yaw_rad']={p:turn(p) for p in ('spin_left','spin_right')}
        (self.out/'smoke.json').write_text(json.dumps(r,indent=2))
        (self.out/'trajectory.json').write_text(json.dumps(self.trace,indent=2))
        return r


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',default='reports/smoke')
    ap.add_argument('--mode',choices=('ackermann','diff'),default='ackermann')
    args,ros=ap.parse_known_args();rclpy.init(args=ros);n=Probe(Path(args.output),args.mode)
    try:
        deadline=time.monotonic()+50
        while not all(k in n.last for k in ('truth','image','info','joints','imu')):
            if time.monotonic()>deadline:raise TimeoutError(f'Missing topics, received {dict(n.counts)}')
            rclpy.spin_once(n,timeout_sec=.1)
        n.run_phase('settle',2)
        n.run_phase('straight',3,.25,0)
        n.run_phase('stop',2)
        n.run_phase('left',3,.2,.3)
        n.run_phase('right',3,.2,-.3)
        n.run_phase('timeout',2,send=False)
        if args.mode == 'diff':
            n.run_phase('spin_left',3,0.,.6)
            n.run_phase('spin_stop',1)
            n.run_phase('spin_right',3,0.,-.6)
            n.run_phase('spin_timeout',2,send=False)
            n.run_phase('final_stop',1)
        r=n.result();print(json.dumps(r,indent=2))
        if not all(r['checks'].values()):raise SystemExit(1)
    finally:
        n.pub.publish(Twist());n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
