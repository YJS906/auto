#!/usr/bin/env python3
"""Restart the complete server for each trial: clears velocities AND plugin odometry.

Source tools/env.sh first. Does not touch any other running Gazebo process.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

ROOT=Path(__file__).resolve().parents[1]


def stop(proc):
    if proc.poll() is not None:return
    os.killpg(proc.pid,signal.SIGINT)
    try:proc.wait(timeout=12)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid,signal.SIGTERM)
        try:proc.wait(timeout=5)
        except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--repeat',type=int,default=2)
    ap.add_argument('--output',type=Path,default=ROOT/'reports/repeat')
    ap.add_argument('--record',action='store_true');args=ap.parse_args()
    results=[]
    for i in range(args.repeat):
        out=args.output/f'trial_{i+1}';out.mkdir(parents=True,exist_ok=True)
        env=dict(os.environ,ROS_DOMAIN_ID='73',GZ_PARTITION=f'autorace_trial_{os.getpid()}_{i}')
        with (out/'sim.log').open('w') as log:
            sim=subprocess.Popen(['ros2','launch','limo_autorace_sim','sim.launch.py','gui:=false'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            bag=None
            try:
                if args.record:
                    bag=subprocess.Popen(['ros2','bag','record','--use-sim-time','-o',str(out/'bag'),
                        '/clock','/cmd_vel','/odom','/joint_states','/tf','/tf_static','/camera/color/image_raw',
                        '/camera/color/camera_info','/imu','/evaluation/ground_truth'],env=env,
                        stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                with (out/'probe.log').open('w') as probe:
                    result=subprocess.run(['ros2','run','limo_autorace_sim','smoke_test','--output',str(out)],
                                          env=env,stdout=probe,stderr=subprocess.STDOUT,timeout=180)
                if result.returncode:raise RuntimeError(f'Trial failed: {out}/probe.log')
                results.append(json.loads((out/'smoke.json').read_text()))
                print(f'Trial {i+1}: PASS, straight {results[-1]["travel_m"]:.4f} m',flush=True)
            finally:
                if bag:stop(bag)
                stop(sim)
    summary=dict(trials=len(results),all_pass=all(all(r['checks'].values()) for r in results),
                 travel_m=[r['travel_m'] for r in results],
                 reset='fresh Gazebo process and ROS nodes; original spawn restored')
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2))
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
