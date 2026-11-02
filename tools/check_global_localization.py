#!/usr/bin/env python3
"""Rendered-camera global startup, blackout and kidnapped-robot regression.

Ground truth is read only by this evaluator. set_pose is a fault injection,
never a localization input. Each run owns an isolated ROS/Gazebo domain.
"""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import platform
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from run_localization_benchmark import ROOT, pose, stamp, stats, stop, wrap


class Check(Node):
    def __init__(self):
        super().__init__('global_localization_check', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.phase = 'startup'
        self.blank = True
        self.ready = False
        self.truth = self.estimate = None
        self.metrics = {}
        self.global_searches, self.last_search = [], -1
        self.states, self.samples = [], []
        self.output = (0., 0.)
        self.command = self.create_publisher(Twist, '/cmd_vel', 10)
        self.images = self.create_publisher(Image, '/evaluation/image', qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/color/image_raw', self.image, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/evaluation/ground_truth', self.ground_truth, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/localization/pose', self.location, 100)
        self.create_subscription(Bool, '/localization/ready', self.readiness,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String, '/localization/state', self.state,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(DiagnosticArray, '/localization/diagnostics', self.diagnostic, 100)
        self.create_subscription(Twist, '/sim/cmd_vel', self.guarded, 100)

    def simtime(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def image(self, msg):
        if self.blank:
            msg = copy.deepcopy(msg)
            msg.data = bytes(len(msg.data))
        self.images.publish(msg)

    def ground_truth(self, msg):
        self.truth = [stamp(msg), *pose(msg.pose.pose)]

    def location(self, msg):
        self.estimate = [stamp(msg), *pose(msg.pose.pose)]
        self.samples.append(dict(phase=self.phase, t=stamp(msg), pose=self.estimate[1:],
                                 truth=self.truth, ready=self.ready))

    def readiness(self, msg):
        self.ready = msg.data

    def state(self, msg):
        if not self.states or self.states[-1]['state'] != msg.data:
            self.states.append(dict(phase=self.phase, t=self.simtime(), wall=time.monotonic(), state=msg.data))
            print(self.states[-1], flush=True)

    def diagnostic(self, msg):
        if msg.status:
            self.metrics = {v.key: v.value for v in msg.status[0].values}
            self.metrics['message'] = msg.status[0].message
            count = int(self.metrics.get('global_searches', 0))
            if count > self.last_search:
                self.last_search = count
                duration = float(self.metrics.get('global_search_ms', 0))
                if duration > 0:
                    self.global_searches.append(duration)

    def guarded(self, msg):
        self.output = (msg.linear.x, msg.angular.z)

    def wait(self, predicate, timeout=90, speed=0.):
        started, last_print = time.monotonic(), -math.inf
        while not predicate():
            if time.monotonic() - started > timeout:
                raise TimeoutError(f'{self.phase}: {self.metrics}')
            cmd = Twist(); cmd.linear.x = float(speed); self.command.publish(cmd)
            rclpy.spin_once(self, timeout_sec=.01)
            if time.monotonic() - last_print > 5:
                print(self.phase, self.simtime(), self.metrics, flush=True)
                last_print = time.monotonic()

    def duration(self, seconds, speed=0.):
        start = self.simtime()
        self.wait(lambda: self.simtime() - start >= seconds, speed=speed)

    def acquire(self, name):
        self.phase = name
        start_sim, start_wall = self.simtime(), time.monotonic()
        self.wait(lambda: self.ready and self.estimate is not None and self.estimate[0] >= start_sim)
        acquisition_sim, acquisition_wall = self.simtime()-start_sim, time.monotonic()-start_wall
        self.duration(1)
        error = math.hypot(self.truth[1]-self.estimate[1], self.truth[2]-self.estimate[2])
        yaw = abs(math.degrees(wrap(self.truth[3]-self.estimate[3])))
        return dict(acquisition_sim_seconds=acquisition_sim, acquisition_wall_seconds=acquisition_wall,
                    settle_sim_seconds=1,
                    position_m=error, yaw_deg=yaw, truth=self.truth, estimate=self.estimate,
                    diagnostic=self.metrics.copy())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--domain', type=int, default=94)
    parser.add_argument('--start', type=float, nargs=3, default=[1.9, 5.57, .05], metavar=('X', 'Y', 'YAW'))
    parser.add_argument('--relocate', type=float, nargs=3, default=[4.5, 4.65, math.pi], metavar=('X', 'Y', 'YAW'))
    args = parser.parse_args()
    args.output = args.output.resolve(); args.output.mkdir(parents=True, exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), GZ_PARTITION=f'global_check_{os.getpid()}')
    processes, logs, node = [], [], None
    if not all(math.isfinite(x) for x in args.start+args.relocate):
        parser.error('Poses must be finite')
    report = dict(host=platform.uname()._asdict(), start=args.start, relocate=args.relocate, checks={}, results={})
    try:
        for name, command in [
            ('sim', ['ros2', 'launch', 'limo_autorace_sim', 'sim.launch.py', 'gui:=false',
                     f'x:={args.start[0]}', f'y:={args.start[1]}', f'yaw:={args.start[2]}', 'require_localization:=true']),
            ('localizer', ['ros2', 'launch', 'limo_localization', 'localization.launch.py',
                           'image:=/evaluation/image'])]:
            log = (args.output / f'{name}.log').open('w'); logs.append(log)
            processes.append(subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                                               start_new_session=True))
        rclpy.init(); node = Check()
        node.wait(lambda: node.truth is not None)
        origin = node.truth.copy()
        node.duration(3, speed=.15)
        report['checks']['blank_start_not_ready'] = not node.ready and node.estimate is None
        report['checks']['guard_blocks_unlocalized_command'] = (
            math.hypot(node.truth[1]-origin[1], node.truth[2]-origin[2]) < .01 and
            max(abs(x) for x in node.output) < .001)
        node.blank = False
        report['results']['unknown_start'] = node.acquire('unknown_start')
        node.phase = 'blackout'; node.blank = True
        node.wait(lambda: not node.ready, timeout=15)
        node.duration(2, speed=.15)
        report['checks']['blackout_loses_readiness'] = not node.ready
        report['checks']['guard_blocks_lost_command'] = max(abs(x) for x in node.output) < .001
        node.blank = False
        report['results']['blackout_recovery'] = node.acquire('blackout_recovery')
        node.phase = 'kidnap'
        x, y, yaw = args.relocate
        request = (f'name: "limo", position: {{x: {x}, y: {y}, z: 0.06}}, '
                   f'orientation: {{z: {math.sin(yaw/2)}, w: {math.cos(yaw/2)}}}')
        injected_sim, injected_wall = node.simtime(), time.monotonic()
        subprocess.run(['gz', 'service', '-s', '/world/autorace/set_pose', '--reqtype', 'gz.msgs.Pose',
                        '--reptype', 'gz.msgs.Boolean', '--timeout', '5000', '--req', request],
                       check=True, timeout=10, capture_output=True, text=True)
        node.wait(lambda: not node.ready, timeout=20)
        report['results']['kidnap_detection'] = dict(sim_seconds=node.simtime()-injected_sim,
                                                    wall_seconds=time.monotonic()-injected_wall)
        report['checks']['kidnap_loses_readiness'] = True
        report['results']['kidnap_recovery'] = node.acquire('kidnap_recovery')
        for name in ('unknown_start', 'blackout_recovery', 'kidnap_recovery'):
            result = report['results'][name]
            report['checks'][f'{name}_below_5cm_2deg'] = result['position_m'] < .05 and result['yaw_deg'] < 2
        node.phase = 'heartbeat_loss'
        node.wait(lambda: abs(node.output[0]) > .05, timeout=10, speed=.15)
        os.killpg(processes[-1].pid, signal.SIGSTOP)
        stopped = time.monotonic()
        node.wait(lambda: time.monotonic()-stopped > 1.2, timeout=5, speed=.15)
        report['checks']['guard_brakes_on_missing_heartbeat'] = max(abs(x) for x in node.output) < .001
    except Exception as exc:
        report['error'] = repr(exc)
        report['checks']['completed'] = False
        print(report['error'], flush=True)
    finally:
        if node:
            report['global_search_wall_ms'] = stats(node.global_searches)
            report['states'] = node.states
            (args.output / 'samples.json').write_text(json.dumps(node.samples, indent=2))
            node.command.publish(Twist()); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        for process in reversed(processes):
            if process.poll() is None: os.killpg(process.pid, signal.SIGCONT)
            stop(process)
        for log in logs: log.close()
        report['limitation'] = ('Static distinctive poses and rendered camera on host; no real lift dynamics, '
                                'full-course coverage or ARM performance certification. set_pose is fault injection only.')
        (args.output / 'summary.json').write_text(json.dumps(report, indent=2, allow_nan=False))
        print(json.dumps(report, indent=2), flush=True)
    return 0 if all(report['checks'].values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
