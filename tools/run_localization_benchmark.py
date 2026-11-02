#!/usr/bin/env python3
"""Compare all four localization variants on identical simulated sensor input.

Starts its own isolated simulation; drives a fixed low-speed forward/reverse
sequence, blanks RGB for two simulated seconds, and optionally biases wheel
speed. Ground truth is subscribed ONLY by this evaluator. No pose teleporting.
Source tools/env.sh after building limo_localization before running.
"""
import argparse
import bisect
import copy
import json
import math
import os
from pathlib import Path
import platform
import signal
import subprocess
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, DurabilityPolicy
import yaml

ROOT = Path(__file__).resolve().parents[1]
VARIANTS = [f'{projection}_{matcher}' for projection in ('ipm', 'rgbd') for matcher in ('distance', 'icp')]


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def pose(msg):
    p, q = msg.position, msg.orientation
    return [p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))]


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def stats(values):
    if not values:
        return None
    a = np.asarray(values, dtype=float)
    return dict(mean=float(a.mean()), rmse=float(np.sqrt(np.mean(a*a))), p50=float(np.percentile(a, 50)),
                p95=float(np.percentile(a, 95)), max=float(a.max()))


class Evaluator(Node):
    def __init__(self, scale):
        super().__init__('localization_evaluator', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.phase = 'startup'
        self.truth, self.estimates, self.metrics = [], {v: [] for v in VARIANTS}, {v: [] for v in VARIANTS}
        self.last_processed = {v: -1 for v in VARIANTS}
        self.ready = {v: False for v in VARIANTS}
        self.availability = {v: {} for v in VARIANTS}
        self.previous_odom = None
        self.wheel_scale = scale
        self.images = 0
        self.command = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_pub = self.create_publisher(Image, '/evaluation/localization_image', qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, '/evaluation/localization_odom', qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/color/image_raw', self.image, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odometry, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/evaluation/ground_truth',
                                 lambda m: self.truth.append([stamp(m), *pose(m.pose.pose)]), qos_profile_sensor_data)
        for variant in VARIANTS:
            self.create_subscription(Bool, f'/{variant}/localization/ready',
                                     lambda m, v=variant: self.ready.__setitem__(v, m.data),
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_subscription(PoseWithCovarianceStamped, f'/{variant}/localization/pose',
                                     lambda m, v=variant: self.estimates[v].append(
                                         dict(t=stamp(m), pose=pose(m.pose.pose), phase=self.phase)), 100)
            self.create_subscription(DiagnosticArray, f'/{variant}/localization/diagnostics',
                                     lambda m, v=variant: self.diagnostic(v, m), 100)

    def image(self, msg):
        self.images += 1
        if self.phase == 'occlusion':
            msg = copy.deepcopy(msg)
            msg.data = bytes(len(msg.data))
        self.image_pub.publish(msg)

    def odometry(self, msg):
        t = stamp(msg)
        dt = 0 if self.previous_odom is None else min(max(t-self.previous_odom, 0), .1)
        self.previous_odom = t
        for variant in VARIANTS:
            phase = self.availability[variant].setdefault(self.phase, [0., 0.])
            phase[0] += dt
            phase[1] += dt * self.ready[variant]
        biased = copy.deepcopy(msg)
        biased.twist.twist.linear.x *= self.wheel_scale
        self.odom_pub.publish(biased)

    def diagnostic(self, variant, msg):
        if not msg.status:
            return
        values = {v.key: float(v.value) for v in msg.status[0].values if v.key != 'state'}
        if values.get('processed', 0) <= self.last_processed[variant]:
            return
        self.last_processed[variant] = values['processed']
        self.metrics[variant].append(dict(wall=time.monotonic(), phase=self.phase,
                                          state=next(v.value for v in msg.status[0].values if v.key == 'state'),
                                          status=msg.status[0].message, **values))

    def run_phase(self, name, duration, speed=0, yaw_rate=0):
        self.phase = name
        previous = self.get_clock().now().nanoseconds * 1e-9
        elapsed, unavailable = 0., 0.
        wall = time.monotonic()
        last_sent = -1e9
        while elapsed < duration:
            if time.monotonic() - wall > 180:
                raise TimeoutError(f'Simulation stalled during {name}')
            now = self.get_clock().now().nanoseconds * 1e-9
            dt, previous = max(now-previous, 0), now
            ready = all(self.ready.values())
            if ready or name == 'occlusion': elapsed += dt
            unavailable = 0 if ready else unavailable+dt
            if name != 'occlusion' and unavailable > 20:
                raise TimeoutError(f'No unique relocalization while stopped in {name}: {self.ready}')
            if time.monotonic() - last_sent >= .05:
                msg = Twist()
                msg.linear.x = float(speed) if ready else 0.
                msg.angular.z = float(yaw_rate) if ready else 0.
                self.command.publish(msg)
                last_sent = time.monotonic()
            rclpy.spin_once(self, timeout_sec=.005)

    def report(self):
        truth = sorted(self.truth)
        times = [t[0] for t in truth]
        results = {}
        for variant in VARIANTS:
            errors = []
            for estimate in self.estimates[variant]:
                t = estimate['t']
                i = bisect.bisect_left(times, t)
                if not 0 < i < len(truth) or truth[i][0] - truth[i - 1][0] > .15:
                    continue
                a, b = truth[i - 1], truth[i]
                fraction = (t - a[0]) / (b[0] - a[0])
                expected = [a[1] + fraction * (b[1] - a[1]), a[2] + fraction * (b[2] - a[2]),
                            wrap(a[3] + fraction * wrap(b[3] - a[3]))]
                p = estimate['pose']
                errors.append(dict(t=t, phase=estimate['phase'],
                                   position_m=math.hypot(p[0] - expected[0], p[1] - expected[1]),
                                   yaw_deg=abs(math.degrees(wrap(p[2] - expected[2])))))
            evaluated = [e for e in errors if e['phase'] not in ('startup', 'settle')]
            metrics = [m for m in self.metrics[variant] if m['phase'] != 'startup']
            last = self.metrics[variant][-1] if self.metrics[variant] else {}
            elapsed = metrics[-1]['wall'] - metrics[0]['wall'] if len(metrics) > 1 else 0
            results[variant] = dict(
                samples=len(evaluated),
                phase_ready_fraction={p: ready/total if total else 0 for p, (total, ready)
                                      in self.availability[variant].items()},
                position_m=stats([e['position_m'] for e in evaluated]),
                yaw_deg=stats([e['yaw_deg'] for e in evaluated]),
                phase_position_m={p: stats([e['position_m'] for e in errors if e['phase'] == p])
                                  for p in sorted({e['phase'] for e in errors})},
                extraction_wall_ms=stats([m['extraction_ms'] for m in metrics]),
                matching_wall_ms=stats([m['matching_ms'] for m in metrics]),
                processing_wall_ms=stats([m['extraction_ms'] + m['matching_ms'] for m in metrics]),
                capture_to_result_sim_ms=stats([m['capture_to_result_sim_ms'] for m in metrics]),
                processing_wall_hz=(len(metrics) - 1) / elapsed if elapsed else 0,
                accepted=int(last.get('accepted', 0)), rejected=int(last.get('rejected', 0)),
                raw_errors=errors, raw_metrics=self.metrics[variant])
        return results


def stop(process):
    if process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True, help='New directory, never overwrites an earlier run')
    parser.add_argument('--domain', type=int, default=93)
    parser.add_argument('--wheel-scale', type=float, default=1.03)
    parser.add_argument('--blank-seconds', type=float, default=2.)
    args = parser.parse_args()
    if not math.isfinite(args.wheel_scale) or not .5 <= args.wheel_scale <= 1.5:
        parser.error('wheel-scale must be between .5 and 1.5')
    if not math.isfinite(args.blank_seconds) or not 0 <= args.blank_seconds <= 10:
        parser.error('blank-seconds must be between 0 and 10')
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), GZ_PARTITION=f'localization_benchmark_{os.getpid()}')
    sensors = yaml.safe_load((ROOT / 'src/limo_autorace_sim/config/sensors.yaml').read_text())
    sensors['depth']['enabled'] = True
    config = args.output / 'sensors.yaml'
    config.write_text(yaml.safe_dump(sensors, sort_keys=False))
    processes, logs = [], []
    node = None
    try:
        def launch(name, command):
            log = (args.output / f'{name}.log').open('w')
            logs.append(log)
            process = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            processes.append(process)

        launch('sim', ['ros2', 'launch', 'limo_autorace_sim', 'sim.launch.py', 'gui:=false', f'sensor_config:={config}'])
        for variant in VARIANTS:
            projection, matcher = variant.split('_')
            launch(variant, ['ros2', 'launch', 'limo_localization', 'localization.launch.py',
                             f'namespace:={variant}', f'projection:={projection}', f'matcher:={matcher}',
                             'publish_tf:=false', 'image:=/evaluation/localization_image',
                             'global_initialization:=false',
                             'odom:=/evaluation/localization_odom'])
        rclpy.init()
        node = Evaluator(args.wheel_scale)
        deadline = time.monotonic() + 90
        while not (node.images > 10 and node.truth and all(node.estimates.values())):
            if time.monotonic() > deadline:
                raise TimeoutError('Missing input/output streams; inspect logs')
            if any(p.poll() is not None for p in processes):
                raise RuntimeError('A launched process exited; inspect logs')
            rclpy.spin_once(node, timeout_sec=.02)
        for phase, duration, speed, yaw_rate in [
                ('settle', 3, 0, 0), ('forward', 3, .25, 0), ('occlusion', args.blank_seconds, .2, 0),
                ('recovery', 4, .2, 0), ('stop', 2, 0, 0),
                ('right_arc', 2, .15, -.25), ('left_arc', 2, .15, .25),
                ('reverse', 8, -.2, 0), ('final_stop', 2, 0, 0)]:
            print(phase, flush=True)
            node.run_phase(phase, duration, speed, yaw_rate)
        results = node.report()
        compact = {k: {field: value for field, value in v.items() if not field.startswith('raw_')}
                   for k, v in results.items()}
        checks = {k: dict(enough_samples=v['samples'] > 100, visual_updates=v['accepted'] > 10,
                          every_motion_phase_has_pose=all(v['phase_position_m'].get(p) is not None
                                                         for p in ('forward','recovery','right_arc','left_arc','reverse')),
                          position_p95_below_10cm=v['position_m'] is not None and v['position_m']['p95'] < .1)
                  for k, v in compact.items()}
        report = dict(host=platform.uname()._asdict(), wheel_speed_scale=args.wheel_scale,
                      initial_prior_error=[.06, -.04, .03], blank_rgb_sim_seconds=args.blank_seconds,
                      control='Stop all variants whenever any ready=false; motion duration excludes pauses. '
                              'Forward segment shortened to retain distinctive markings at blackout stop.',
                      limitation='Short forward/reverse segment, not full-course or target-hardware certification. '
                                 'Four nodes run concurrently; throughput includes simulator and competitor load. '
                                 'Map coordinate origin equals Gazebo world origin; truth is timestamp-interpolated.',
                      results=compact, checks=checks)
        (args.output / 'samples.json').write_text(json.dumps(results, indent=2, allow_nan=False))
        (args.output / 'summary.json').write_text(json.dumps(report, indent=2, allow_nan=False))
        print(json.dumps(report, indent=2))
        return 0 if all(all(c.values()) for c in checks.values()) else 1
    finally:
        if node:
            if not (args.output / 'summary.json').exists():
                results = node.report()
                (args.output / 'samples.json').write_text(json.dumps(results, indent=2, allow_nan=False))
                compact = {k: {f: v for f, v in value.items() if not f.startswith('raw_')}
                           for k, value in results.items()}
                (args.output / 'summary.json').write_text(json.dumps(dict(
                    completed=False, phase=node.phase, ready=node.ready, results=compact,
                    limitation='Incomplete run: see evaluator stderr for failure; poses absent while unlocalized.'), indent=2))
            node.command.publish(Twist())
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for process in reversed(processes):
            stop(process)
        for log in logs:
            log.close()


if __name__ == '__main__':
    raise SystemExit(main())
