#!/usr/bin/env python3
"""Exercise the unchanged LIMO driver against a private pseudo-terminal, never hardware.

Source ROS and the audit install first. Every command is isolated to ROS_DOMAIN_ID=184.
VTune raw databases remain in /tmp; JSON/log/CSV evidence goes beside this script.
"""
import argparse
import collections
import fcntl
import json
import math
import os
from pathlib import Path
import pty
import resource
import signal
import struct
import subprocess
import time


def frame(mid, payload, count=0):
    assert len(payload) == 8
    return bytes((0x55, 14, mid >> 8, mid & 255)) + payload + bytes((count, sum(payload) & 255))


def process_tree(pid):
    result = [pid]
    try:
        for child in Path(f'/proc/{pid}/task/{pid}/children').read_text().split():
            result.extend(process_tree(int(child)))
    except FileNotFoundError:
        pass
    return result


def stats(pid):
    try:
        fields = Path(f'/proc/{pid}/stat').read_text().split()
        return {'cpu_s': (int(fields[13]) + int(fields[14])) / os.sysconf('SC_CLK_TCK'),
                'rss_kib': int(fields[23]) * os.sysconf('SC_PAGE_SIZE') / 1024}
    except FileNotFoundError:
        return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', choices=['normal', 'stop', 'invalid', 'disconnect', 'watchdog', 'checksum', 'mixed_imu', 'reverse'], default='normal')
    parser.add_argument('--duration', type=float, default=6)
    parser.add_argument('--rate', type=int, default=50)
    parser.add_argument('--profile', choices=['none', 'vtune', 'perf', 'strace'], default='none')
    parser.add_argument('--tag', default='normal')
    parser.add_argument('--binary', default='/tmp/limo-driver-audit/install/limo_base/lib/limo_base/limo_base')
    args = parser.parse_args()
    domain_lock = open('/tmp/limo-driver-audit/domain184.lock', 'w')
    fcntl.flock(domain_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    out = Path(__file__).resolve().parent
    assert args.rate > 0 and args.duration > 0
    os.environ['ROS_DOMAIN_ID'] = '184'
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ['ROS_LOG_DIR'] = '/tmp/limo-driver-audit/ros_logs'
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from limo_msgs.msg import LimoStatus

    master, slave = pty.openpty()
    alias = Path(f'/dev/shm/ttyLimoAudit_{os.getpid()}')
    alias.symlink_to(os.ttyname(slave))
    os.set_blocking(master, False)
    command = [args.binary, '--ros-args', '-p', f'port_name:=shm/{alias.name}']
    raw_result = f'/tmp/limo-driver-audit/vtune_{args.tag}'
    if args.profile == 'vtune':
        command = ['vtune', '-collect', 'hotspots', '-knob', 'sampling-mode=sw',
                   '-knob', 'enable-characterization-insights=false', '-result-dir', raw_result, '--'] + command
    elif args.profile == 'perf':
        command = ['/usr/lib/linux-tools/7.0.0-38-generic/perf', 'stat', '-e',
                   'task-clock,context-switches,cycles,instructions', '-o', str(out / f'{args.tag}.perf.txt'), '--'] + command
    elif args.profile == 'strace':
        command = ['strace', '-f', '-c', '-o', str(out / f'{args.tag}.strace.txt'), '--'] + command

    rclpy.init()
    node = rclpy.create_node('limo_audit_probe')
    samples = collections.defaultdict(list)
    sent_times = {}
    imu_latencies_ms = []
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    def stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    def odom(msg):
        q = msg.pose.pose.orientation
        samples['odom'].append({'received': time.time(), 'stamp': stamp(msg),
             'x': msg.pose.pose.position.x, 'y': msg.pose.pose.position.y,
             'yaw_deg': math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))),
             'vx': msg.twist.twist.linear.x, 'vy': msg.twist.twist.linear.y,
             'wz': msg.twist.twist.angular.z,
             'twist_covariance': list(msg.twist.covariance)})
    def imu(msg):
        q = msg.orientation
        index = round(math.degrees(2 * math.atan2(q.z, q.w)) / 0.05)
        if index in sent_times:
            imu_latencies_ms.append(1000 * (time.monotonic() - sent_times[index]))
        samples['imu'].append({'received': time.time(), 'stamp': stamp(msg),
            'quaternion': [q.x, q.y, q.z, q.w], 'gyro_z': msg.angular_velocity.z,
            'accel_z': msg.linear_acceleration.z})
    def status(msg):
        samples['status'].append({'received': time.time(), 'stamp': stamp(msg),
            'battery': msg.battery_voltage, 'mode': msg.motion_mode})
    subscriptions = [node.create_subscription(Odometry, '/odom', odom, 1000),
                     node.create_subscription(Imu, '/imu', imu, 1000),
                     node.create_subscription(LimoStatus, '/limo_status', status, 1000)]
    start = time.monotonic()
    log_path = out / f'{args.tag}.log'
    target_pid = None
    tx = bytearray()
    proc_stats = []
    bytes_sent = 0
    injected_commands = []
    shutdown_error = None
    with log_path.open('w') as log:
        proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        try:
            next_tick = start + 1.5
            end = start + 20.0  # startup timeout; data window starts after serial is open
            input_start = None
            next_command = float('inf')
            index = 0
            disconnected = False
            while time.monotonic() < end and proc.poll() is None:
                now = time.monotonic()
                if target_pid is None:
                    for candidate in process_tree(proc.pid):
                        try:
                            if Path(f'/proc/{candidate}/comm').read_text().strip() == 'limo_base':
                                target_pid = candidate
                        except FileNotFoundError:
                            pass
                if target_pid:
                    sample = stats(target_pid)
                    if sample:
                        proc_stats.append({'elapsed': now-start, **sample})
                if args.case == 'disconnect' and input_start is not None and now > input_start + args.duration / 2 and not disconnected:
                    os.close(master)
                    master = -1
                    disconnected = True
                    disconnect_at = now-start
                if input_start is not None and now >= next_tick and master >= 0:
                    # One cycle: acceleration, gyro, Euler, motion; system state at rate/5.
                    yaw = round(index * 0.05 * 100)
                    speed = -200 if args.case == 'reverse' and index > args.duration * args.rate / 2 else 200
                    packets = [frame(0x321, struct.pack('>hhhh', 0, 0, 981, 0)),
                               frame(0x322, struct.pack('>hhhh', 0, 0, 250, 0)),
                               frame(0x323, struct.pack('>hhhh', yaw, 500, 1000, 0)),
                               frame(0x221, struct.pack('>hhhh', speed, 150, 0, 80))]
                    if args.case == 'mixed_imu' and index > 10:
                        packets = packets[2:]
                    if index % 5 == 0:
                        packets.insert(0, frame(0x211, bytes((0, 0, 0, 120, 0, 0, 1, 0))))
                    if args.case == 'checksum':
                        packets = [p[:-1] + bytes((p[-1] ^ 1,)) for p in packets]
                    payload = b''.join(packets)
                    try:
                        sent_times[index] = time.monotonic()
                        bytes_sent += os.write(master, payload)
                    except BlockingIOError:
                        pass
                    index += 1
                    next_tick += 1 / args.rate
                if args.case in ('stop', 'invalid', 'watchdog') and now > next_command and pub.get_subscription_count():
                    msg = Twist()
                    if args.case == 'invalid':
                        msg.linear.x = float('nan')
                        msg.angular.z = float('inf')
                    elif args.case == 'watchdog':
                        msg.linear.x = 0.2
                        msg.angular.z = 0.1
                    pub.publish(msg)
                    injected_commands.append({'elapsed': now-start, 'v': str(msg.linear.x), 'w': str(msg.angular.z)})
                    next_command = end + 100
                if master >= 0:
                    try:
                        tx.extend(os.read(master, 65536))
                    except BlockingIOError:
                        pass
                    # The original constructor sends this only after openPort succeeds.
                    # Avoid losing initial injected frames to tcflush during profiler startup.
                    if input_start is None and bytes((0x55, 14, 4, 0x21)) in tx:
                        input_start = max(time.monotonic() + 0.05, start + 1.5)
                        next_tick = input_start
                        end = input_start + args.duration
                        next_command = input_start + 1.0
                for _ in range(4):
                    rclpy.spin_once(node, timeout_sec=0)
                time.sleep(min(0.001, 1 / args.rate / 2))
            # Let already-written frames reach DDS subscribers before counting delivery.
            drain_until = time.monotonic() + 0.25
            while master >= 0 and proc.poll() is None and time.monotonic() < drain_until:
                rclpy.spin_once(node, timeout_sec=0.005)
            if target_pid and Path(f'/proc/{target_pid}').exists():
                os.kill(target_pid, signal.SIGINT)
            elif proc.poll() is None:
                proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=30)
            except subprocess.TimeoutExpired:
                shutdown_error = 'target/profile did not terminate within 30 seconds'
                for pid in reversed(process_tree(proc.pid)):
                    try:
                        os.kill(pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                proc.wait()
        finally:
            if master >= 0:
                os.close(master)
            os.close(slave)
            alias.unlink(missing_ok=True)
            node.destroy_node()
            rclpy.shutdown()
    decoded = []
    for i in range(len(tx)-13):
        if tx[i:i+2] == bytes((0x55, 14)):
            b = tx[i:i+14]
            decoded.append({'id': hex(int.from_bytes(b[2:4], 'big')),
                            'values_i16': list(struct.unpack('>hhhh', b[4:12])),
                            'count': b[12], 'checksum_valid': b[13] == (sum(b[4:12]) & 255)})
    counts = {k: len(v) for k, v in samples.items()}
    summary = {'case': args.case, 'profile': args.profile, 'rate_hz': args.rate,
        'duration_s': args.duration, 'command': command, 'target_pid': target_pid,
        'input_start_elapsed_s': input_start-start if input_start is not None else None,
        'returncode': proc.returncode, 'shutdown_error': shutdown_error,
        'bytes_sent': bytes_sent, 'cycles_sent': index, 'messages_received': counts, 'outbound_frames': decoded,
        'injected_commands': injected_commands, 'process_stats': proc_stats[::20],
        'last_process_stats': proc_stats[-1] if proc_stats else None,
        'samples_first_last': {k: {'first': v[0], 'last': v[-1]} for k, v in samples.items() if v}}
    if args.case == 'disconnect' and disconnected:
        summary['disconnect_at_elapsed_s'] = disconnect_at
    if args.profile == 'vtune':
        summary['vtune_raw_result'] = raw_result
    if imu_latencies_ms:
        ordered = sorted(imu_latencies_ms)
        summary['synthetic_send_to_imu_callback_ms'] = {
            'count': len(ordered), 'median': ordered[len(ordered)//2],
            'p95': ordered[min(len(ordered)-1, math.ceil(0.95*len(ordered))-1)],
            'max': ordered[-1]}
    (out / f'{args.tag}.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps({k: summary[k] for k in ('case', 'profile', 'returncode', 'messages_received', 'last_process_stats', 'shutdown_error')}, indent=2))


if __name__ == '__main__':
    main()
