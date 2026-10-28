import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
from .control import Envelope


class CommandGuard(Node):
    def __init__(self):
        super().__init__('command_guard')
        self.env = Envelope()
        for key in ('mode','wheelbase','central_steering_limit','max_speed','max_yaw_rate',
                    'acceleration','deceleration','steering_rate'):
            setattr(self.env, key, self.declare_parameter(key, getattr(self.env,key)).value)
        if self.env.mode not in ('ackermann', 'diff'):
            raise ValueError('mode must be ackermann or diff')
        self.timeout = self.declare_parameter('command_timeout', 0.35).value
        rate = self.declare_parameter('rate', 50.0).value
        self.target = (0.0, 0.0)
        self.received = -math.inf
        self.received_wall = -math.inf
        self.previous = None
        self.estop = False
        self.pub = self.create_publisher(Twist, '/sim/cmd_vel', 10)
        self.create_subscription(Twist, 'cmd_vel', self.command, 10)
        self.create_service(SetBool, '~/emergency_stop', self.stop)
        self.create_timer(1/rate, self.tick)

    def command(self, msg):
        if not math.isfinite(msg.linear.x) or not math.isfinite(msg.angular.z):
            self.target = (0.0, 0.0)
            return
        self.target = (msg.linear.x, msg.angular.z)
        self.received = self.get_clock().now().nanoseconds*1e-9
        self.received_wall = time.monotonic()

    def stop(self, req, res):
        self.estop = req.data
        self.target = (0.0, 0.0)
        self.received = -math.inf
        res.success = True
        res.message = 'Emergency stop latched' if req.data else 'Released; send a fresh command'
        return res

    def tick(self):
        now = self.get_clock().now().nanoseconds*1e-9
        if self.previous is None or now < self.previous:
            self.env.reset()
            self.target = (0.0, 0.0)
            self.received = -math.inf
            self.previous = now
        dt = min(max(now-self.previous, 0.0), 0.1)
        self.previous = now
        stale = now-self.received > self.timeout or time.monotonic()-self.received_wall > self.timeout
        target = (0.0, 0.0) if stale or self.estop else self.target
        if self.estop:
            self.env.reset()
        v,w = self.env.step(*target, dt)
        msg = Twist(); msg.linear.x=v; msg.angular.z=w
        self.pub.publish(msg)


def main():
    rclpy.init(); node = CommandGuard()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if rclpy.ok(): node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
