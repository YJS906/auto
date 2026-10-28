#!/usr/bin/env python3
"""Exercise enabled depth/LiDAR at the default spawn, then add a known target.

Run only against a dedicated, stationary simulation. The target is left in the
world for RViz inspection; restarting the simulator removes it. Default sensor
geometry and topic names are intentional expectations, not auto-derived truth.
"""
import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import subprocess
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from tf2_msgs.msg import TFMessage


class SensorProbe(Node):
    def __init__(self, output):
        super().__init__('optional_sensor_probe')
        self.output = output
        self.last = {}
        self.stamps = defaultdict(list)
        self.frames = set()
        for topic, typ, key in [
            ('/camera/depth/image_raw', Image, 'depth'),
            ('/camera/depth/camera_info', CameraInfo, 'info'),
            ('/scan', LaserScan, 'scan'),
            ('/camera/color/image_raw', Image, 'rgb'),
        ]:
            self.create_subscription(typ, topic, lambda m, k=key: self.receive(k, m), qos_profile_sensor_data)
        self.create_subscription(TFMessage, '/tf_static', self.transforms,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def receive(self, key, msg):
        self.last[key] = msg
        self.stamps[key].append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    def transforms(self, msg):
        self.frames.update((t.header.frame_id, t.child_frame_id) for t in msg.transforms)

    def collect(self, seconds=3):
        # Sensor timestamps measure simulated duration independently of host load.
        self.stamps.clear()
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.1)
            if all(len(self.stamps[k]) > 10 and self.stamps[k][-1] - self.stamps[k][0] >= seconds
                   for k in ('depth', 'info', 'scan', 'rgb')):
                return
        raise TimeoutError(f'Missing/slow sensor streams: {dict((k, len(v)) for k, v in self.stamps.items())}')

    def snapshot(self, name):
        msg = self.last['depth']
        depth = CvBridge().imgmsg_to_cv2(msg, desired_encoding='passthrough')
        scan = self.last['scan']
        ranges = np.asarray(scan.ranges)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        front = ranges[np.abs(angles) < math.radians(20)]
        central = ranges[np.abs(angles) < math.radians(3)]
        patch = depth[117:123, 157:163]
        valid = np.isfinite(patch) & (patch > 0)
        np.savez_compressed(self.output / f'{name}_depth.npz', depth=depth)
        preview = np.nan_to_num(depth, nan=0, posinf=0, neginf=0)
        preview = (np.clip(preview, 0, 5) / 5 * 255).astype(np.uint8)
        cv2.imwrite(str(self.output / f'{name}_depth.png'), preview)
        cv2.imwrite(str(self.output / f'{name}_rgb.png'),
                    CvBridge().imgmsg_to_cv2(self.last['rgb'], 'bgr8'))
        (self.output / f'{name}_scan.json').write_text(json.dumps({
            'angle_min': scan.angle_min, 'angle_increment': scan.angle_increment,
            'ranges_m': [float(x) if np.isfinite(x) else None for x in ranges],
        }, indent=2))
        return {
            'encoding': msg.encoding, 'width': msg.width, 'height': msg.height,
            'depth_frame': msg.header.frame_id, 'info_frame': self.last['info'].header.frame_id,
            'scan_frame': scan.header.frame_id, 'scan_samples': len(ranges),
            'depth_finite_fraction': float(np.isfinite(depth).mean()),
            'centre_depth_m': float(np.median(patch[valid])) if valid.any() else None,
            'front_returns': int(np.count_nonzero(np.isfinite(front) & (front < scan.range_max - .05))),
            'centre_scan_m': float(np.median(central[np.isfinite(central)])) if np.isfinite(central).any() else None,
            'rates_hz': {k: float(1 / np.median(np.diff(v))) for k, v in self.stamps.items()},
            'counts': {k: len(v) for k, v in self.stamps.items()},
            'timestamps_increase': all(np.all(np.diff(v) > 0) for v in self.stamps.values()),
            'matched_depth_info_stamps': len(set(self.stamps['depth']) & set(self.stamps['info'])),
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='reports/extended/sensors')
    args = parser.parse_args()
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = SensorProbe(output)
    try:
        node.collect()
        baseline = node.snapshot('baseline')
        fixture = '''<sdf version="1.9"><model name="sensor_validation_target">
<static>true</static><pose>2.6 5.57 0.5 0 0 0</pose><link name="box">
<collision name="box"><geometry><box><size>0.2 0.8 1</size></box></geometry></collision>
<visual name="box"><geometry><box><size>0.2 0.8 1</size></box></geometry>
<material><diffuse>0.8 0.15 0.1 1</diffuse></material></visual>
</link></model></sdf>'''
        (output / 'target.sdf').write_text(fixture)
        result = subprocess.run([
            'gz', 'service', '-s', '/world/autorace/create', '--reqtype', 'gz.msgs.EntityFactory',
            '--reptype', 'gz.msgs.Boolean', '--timeout', '5000', '--req', 'sdf: ' + json.dumps(fixture),
        ], capture_output=True, text=True, timeout=15)
        (output / 'spawn.log').write_text(result.stdout + result.stderr)
        if result.returncode or 'data: true' not in result.stdout:
            raise RuntimeError('Failed to spawn sensor validation target: ' + result.stdout + result.stderr)
        # First collection allows rendering to see the newly added visual.
        node.collect(1)
        node.collect()
        target = node.snapshot('target')
        info = node.last['info']
        scan = node.last['scan']
        expected_f = 320 / (2 * math.tan(1.2 / 2))
        checks = {
            'depth_32FC1_320x240': all(s['encoding'] == '32FC1' and s['width'] == 320 and s['height'] == 240 for s in (baseline, target)),
            'depth_info_dimensions_intrinsics': info.width == 320 and info.height == 240 and abs(info.k[0] - expected_f) < 1 and abs(info.k[4] - expected_f) < 1,
            'optical_and_laser_frames': all(s['depth_frame'] == s['info_frame'] == 'depth_optical_frame' and s['scan_frame'] == 'laser_link' for s in (baseline, target)),
            'static_tf_connected': {('base_link', 'depth_link'), ('depth_link', 'depth_optical_frame'), ('base_link', 'laser_link')} <= node.frames,
            'depth_info_capture_stamps_match': min(baseline['matched_depth_info_stamps'], target['matched_depth_info_stamps']) >= 10,
            'sensor_rates': all(8 <= s['rates_hz'][k] <= 12 for s in (baseline, target) for k in ('depth', 'info', 'scan')),
            'timestamps_increase': baseline['timestamps_increase'] and target['timestamps_increase'],
            'scan_360_samples_and_full_circle': len(scan.ranges) == 360 and abs(scan.angle_min + math.pi) < .01 and abs(scan.angle_max - math.pi) < .01 and abs(359 * scan.angle_increment - 2 * math.pi) < .01,
            'scan_range_limits': abs(scan.range_min - .1) < .001 and abs(scan.range_max - 12) < .001,
            'flat_track_no_forward_lidar_obstacles': baseline['front_returns'] == 0,
            'depth_observes_floor': baseline['centre_depth_m'] is not None and abs(baseline['centre_depth_m'] - .275 / math.sin(.25)) < .08,
            'lidar_measures_target': target['centre_scan_m'] is not None and abs(target['centre_scan_m'] - .8) < .06,
            'depth_measures_target': target['centre_depth_m'] is not None and abs(target['centre_depth_m'] - .72 / math.cos(.25)) < .06,
            'depth_changes_for_target': baseline['centre_depth_m'] is not None and target['centre_depth_m'] is not None and baseline['centre_depth_m'] - target['centre_depth_m'] > .2,
        }
        report = dict(checks={k: bool(v) for k, v in checks.items()}, baseline=baseline, target=target,
                      tf=sorted(node.frames), limitation='Default spawn and sensor geometry only; no real sensor calibration.')
        (output / 'sensors.json').write_text(json.dumps(report, indent=2, allow_nan=False))
        print(json.dumps(report, indent=2, allow_nan=False))
        return 0 if all(checks.values()) else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
