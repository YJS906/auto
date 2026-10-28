#!/usr/bin/env python3
"""Run isolated diff, Ackermann regression and optional sensor checks.

Source tools/env.sh first. Each check starts and stops its own Gazebo server.
Use --nvidia to select NVIDIA EGL explicitly, otherwise use system defaults.
"""
import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time

import yaml


ROOT = Path(__file__).resolve().parents[1]


def stop(process):
    if process.poll() is not None:
        return
    # ROS launch forwards SIGINT itself; signalling the whole group would send
    # each child a second interrupt while it is cleaning up its ROS context.
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=20)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=10)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--checks', nargs='+', choices=['diff', 'ackermann', 'sensors'],
                        default=['diff', 'ackermann', 'sensors'])
    parser.add_argument('--output', type=Path, default=ROOT / 'reports/extended')
    parser.add_argument('--domain', type=int, default=75)
    parser.add_argument('--nvidia', action='store_true')
    args = parser.parse_args()
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID=str(args.domain), GZ_PARTITION=f'autorace_checks_{os.getpid()}')
    if args.nvidia:
        vendor = '/usr/share/glvnd/egl_vendor.d/10_nvidia.json'
        if not Path(vendor).exists():
            parser.error('NVIDIA EGL vendor configuration is missing')
        env.update(__EGL_VENDOR_LIBRARY_FILENAMES=vendor,
                   __NV_PRIME_RENDER_OFFLOAD='1', __GLX_VENDOR_LIBRARY_NAME='nvidia')
    summary = {'kernel': os.uname().release, 'nvidia_requested': args.nvidia, 'results': {}}
    for name in args.checks:
        out = args.output / name
        out.mkdir(parents=True, exist_ok=True)
        mode = 'diff' if name == 'diff' else 'ackermann'
        launch = ['ros2', 'launch', 'limo_autorace_sim', 'sim.launch.py', 'gui:=false', f'mode:={mode}']
        if name == 'sensors':
            sensors = yaml.safe_load((ROOT / 'src/limo_autorace_sim/config/sensors.yaml').read_text())
            sensors['depth']['enabled'] = sensors['lidar']['enabled'] = True
            config = out / 'enabled_sensors.yaml'
            config.write_text(yaml.safe_dump(sensors, sort_keys=False))
            launch.append(f'sensor_config:={config}')
            probe = ['/usr/bin/python3', str(ROOT / 'tools/check_optional_sensors.py'), '--output', str(out)]
            report_name = 'sensors.json'
        else:
            probe = ['ros2', 'run', 'limo_autorace_sim', 'smoke_test', '--mode', mode, '--output', str(out)]
            report_name = 'smoke.json'
        # Never mistake a report from an older run for this run's result.
        (out / report_name).unlink(missing_ok=True)
        print(f'Starting {name}', flush=True)
        with (out / 'sim.log').open('w') as log:
            sim = subprocess.Popen(launch, cwd=ROOT, env=env, stdout=log,
                                   stderr=subprocess.STDOUT, start_new_session=True)
            try:
                with (out / 'probe.log').open('w') as probe_log:
                    result = subprocess.run(probe, cwd=ROOT, env=env, stdout=probe_log,
                                            stderr=subprocess.STDOUT, timeout=180)
                report = json.loads((out / report_name).read_text()) if (out / report_name).exists() else {}
                checks = report.get('checks', {})
                summary['results'][name] = dict(exit_code=result.returncode, checks=checks,
                                                passed=result.returncode == 0 and bool(checks) and all(checks.values()))
                if shutil.which('nvidia-smi'):
                    gpu = subprocess.run(['nvidia-smi'], capture_output=True, text=True, timeout=15)
                    (out / 'nvidia-smi.txt').write_text(gpu.stdout + gpu.stderr)
                ogre = Path.home() / '.gz/rendering/ogre2.log'
                if ogre.exists():
                    shutil.copy2(ogre, out / 'ogre2.log')
            except (subprocess.TimeoutExpired, OSError, ValueError) as error:
                summary['results'][name] = dict(passed=False, error=str(error))
            finally:
                stop(sim)
        print(f'{name}: {summary["results"][name]}', flush=True)
        (args.output / 'summary.json').write_text(json.dumps(summary, indent=2))
        time.sleep(1)
    return 0 if all(r['passed'] for r in summary['results'].values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
