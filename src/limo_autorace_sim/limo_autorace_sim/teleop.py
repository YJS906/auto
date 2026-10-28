"""Deadman keyboard teleop: keys expire after 0.25 wall seconds."""
import select
import sys
import termios
import time
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def main():
    rclpy.init(); n=Node('autorace_teleop')
    pub=n.create_publisher(Twist,'cmd_vel',10)
    speed=n.declare_parameter('speed',0.25).value
    yaw=n.declare_parameter('yaw_rate',0.35).value
    if not sys.stdin.isatty(): raise RuntimeError('Run teleop in an interactive terminal')
    settings=termios.tcgetattr(sys.stdin)
    print('Hold/repeat: w forward, s reverse, a forward-left, d forward-right; SPACE stop; q quit')
    last=0; v=w=0.0
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            if select.select([sys.stdin],[],[],0.05)[0]:
                key=sys.stdin.read(1)
                if key in ('q','\x03'): break
                v,w={'w':(speed,0.0),'s':(-speed,0.0),'a':(speed,yaw),'d':(speed,-yaw)}.get(key,(0.0,0.0))
                last=time.monotonic()
            msg=Twist()
            if time.monotonic()-last < 0.25: msg.linear.x=v; msg.angular.z=w
            pub.publish(msg)
            rclpy.spin_once(n,timeout_sec=0)
    finally:
        for _ in range(3): pub.publish(Twist()); time.sleep(.02)
        termios.tcsetattr(sys.stdin,termios.TCSADRAIN,settings)
        n.destroy_node(); rclpy.shutdown()
