#!/usr/bin/env python3
"""Check that recorded evidence supports the audit; this is not a driver pass suite."""
import hashlib
import json
import math
from pathlib import Path


def main():
    root = Path(__file__).resolve().parent
    def read(name):
        return json.loads((root / f'{name}.json').read_text())
    checks = []
    identity = read('identity')
    source = Path('/home/a/2026autorace/vendor/limo_pro/limo_ros2/limo_base/src/limo_driver.cpp')
    assert identity['local_git_github_identical']
    assert hashlib.sha256(source.read_bytes()).hexdigest() == identity['sha256']
    checks.append('local source still equals verified GitHub object')

    assert '392:43: runtime error: nan' in (root / 'ubsan_stop.log').read_text()
    assert '389:37: runtime error: nan' in (root / 'ubsan_invalid.log').read_text()
    checks.append('stop and invalid command float-cast UB reproduced')

    nominal = read('nominal_50')
    assert nominal['returncode'] == -6
    checks.append('normal SIGINT shutdown abort reproduced')
    status = nominal['samples_first_last']['status']['first']
    assert abs(status['stamp'] * 1e9 - status['received']) < 3
    checks.append('status seconds/nanoseconds scale error reproduced')

    odom = nominal['samples_first_last']['odom']['last']
    imu = nominal['samples_first_last']['imu']['last']
    yaw = math.degrees(2 * math.atan2(imu['quaternion'][2], imu['quaternion'][3]))
    assert abs(yaw - 14.95) < 1e-6 and abs(odom['yaw_deg']) < 1e-6
    checks.append('small yaw increments lost in odometry')
    assert imu['quaternion'][:2] == [0.0, 0.0]
    assert all(x == 0.0 for x in odom['twist_covariance'])
    checks.append('roll/pitch omitted and twist covariance left zero')

    reverse = read('reverse')['samples_first_last']['odom']
    assert reverse['first']['vx'] > 0 > reverse['last']['vx']
    assert reverse['first']['vy'] == reverse['last']['vy'] > 0
    checks.append('reverse lateral velocity sign asymmetry reproduced')

    disconnect = read('disconnect_plain')
    after = [x for x in disconnect['process_stats'] if x['elapsed'] > disconnect['disconnect_at_elapsed_s'] + 0.3]
    cpu_percent = 100 * (after[-1]['cpu_s']-after[0]['cpu_s']) / (after[-1]['elapsed']-after[0]['elapsed'])
    assert cpu_percent > 90
    checks.append('disconnect busy-loop CPU above 90% of one core')

    frames = read('watchdog')['outbound_frames']
    assert len([f for f in frames if f['id'] == '0x111']) == 1
    checks.append('no additional stop frame during command silence')
    assert read('checksum')['messages_received'] == {}
    checks.append('bad checksum frames rejected')

    mixed = read('mixed_imu')
    assert mixed['messages_received']['imu'] == 200
    assert mixed['bytes_sent'] < 11760
    checks.append('IMU keeps publishing after accel/gyro frames stop')

    for rate in (50, 500):
        for kind in ('original', 'batch'):
            for repeat in range(1, 4):
                run = read(f'ready_{kind}_{rate}_r{repeat}')
                assert run['input_start_elapsed_s'] is not None
                # A time-bounded Python feeder can send a few cycles fewer than planned.
                # Verify actual bytes written, not requested rate * duration.
                cycles = [n for n in range(rate*6-30, rate*6+1)
                          if 14*(4*n + (n+4)//5) == run['bytes_sent']]
                assert len(cycles) == 1
                n = cycles[0]
                assert run['messages_received']['imu'] == n
                assert run['messages_received']['odom'] == n
                assert run['messages_received']['status'] == (n+4)//5
                state = run['samples_first_last']
                assert abs(state['odom']['last']['vx'] - 0.19668601732951652) < 1e-12
                assert abs(state['odom']['last']['wz'] - 0.18434233162876418) < 1e-12
                q = state['imu']['last']['quaternion']
                assert abs(math.degrees(2*math.atan2(q[2], q[3])) - (n-1)*0.05) < 1e-6
    checks.append('all 12 synchronized benchmark runs delivered every injected output')
    (root / 'evidence_checks.json').write_text(json.dumps({'checked': len(checks), 'checks': checks}, indent=2) + '\n')
    print(f'{len(checks)} evidence checks passed; known driver defects remain present.')


if __name__ == '__main__':
    main()
