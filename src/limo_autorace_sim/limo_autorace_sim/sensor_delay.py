"""Bounded simulation-time transport delay, preserving sensor capture timestamps."""
from collections import deque
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, LaserScan, Imu


class SensorDelay(Node):
    def __init__(self):
        super().__init__('sensor_delay')
        self.queue = []
        self.maximum = self.declare_parameter('max_queue',150).value
        delays = {k: self.declare_parameter(k+'_delay',0.0).value for k in ('rgb','depth','lidar','imu')}
        if any(v < 0 for v in delays.values()):
            raise ValueError('Delays must be nonnegative')
        self.last = 0.0
        for typ,src,dst,key in [
            (Image,'/sim/raw/image','image','rgb'),
            (CameraInfo,'/sim/raw/camera_info','camera_info','rgb'),
            (Image,'/sim/raw/depth','depth','depth'),
            (CameraInfo,'/sim/raw/depth_info','depth_info','depth'),
            (LaserScan,'/sim/raw/scan','scan','lidar'),
            (Imu,'/sim/raw/imu','imu','imu')]:
            pub = self.create_publisher(typ,dst,qos_profile_sensor_data)
            queue = deque(maxlen=self.maximum)
            self.queue.append((queue,pub))
            def receive(msg,q=queue,p=pub,delay=delays[key]):
                if delay == 0:
                    p.publish(msg)
                else:
                    q.append((self.get_clock().now().nanoseconds*1e-9+delay,msg))
            self.create_subscription(typ,src,receive,qos_profile_sensor_data)
        self.create_timer(0.005,self.flush)

    def flush(self):
        now = self.get_clock().now().nanoseconds*1e-9
        for q,pub in self.queue:
            if now < self.last: q.clear()
            while q and q[0][0] <= now:
                pub.publish(q.popleft()[1])
        self.last=now


def main():
    rclpy.init(); n=SensorDelay()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally:
        n.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
