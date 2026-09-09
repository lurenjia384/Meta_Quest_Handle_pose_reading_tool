#!/usr/bin/env python3
import sys
import types
import time
import argparse

import numpy as np

if 'IPython' not in sys.modules:
    _stub = types.ModuleType('IPython')
    _stub.embed = lambda *a, **k: None
    _stub.get_ipython = lambda: None
    sys.modules['IPython'] = _stub

from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.robots.so_follower.config_so_follower import SO101FollowerConfig
from lerobot.robots.so_follower.so_follower import SO101Follower

MOTOR_NAMES = [
    "shoulder_pan", "shoulder_lift", "elbow_flex",
    "wrist_flex", "wrist_roll", "gripper",
]


def connect_robot(port):
    cfg = SO101FollowerConfig(
        port=port,
        id="my_awesome_follower_arm",
        use_degrees=True,
        cameras={},
    )
    robot = SO101Follower(cfg)
    robot.connect(calibrate=False)
    print(f"Robot connected: {port}")
    return robot


def read_qpos(robot):
    obs = robot.get_observation()
    qpos = np.array([obs[f"{m}.pos"] for m in MOTOR_NAMES], dtype=np.float64)
    return qpos


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='/dev/ttyACM0')
    parser.add_argument('--no-move', action='store_true',
                        help='only read, do not send actions')
    parser.add_argument('--step', type=float, default=20.0,
                        help='per-joint test step in degrees')
    args = parser.parse_args()

    robot = connect_robot(args.port)

    print("\n=== Reading initial state ===")
    q0 = read_qpos(robot)
    print(f"  qpos: {q0.round(2)}")
    print(f"  joint ranges (deg): "
          f"pan±110 lift±100 elbow±96.8 wflex±95 wroll[-157,163] grip[-10,100]")
    print(f"  Motor names: {MOTOR_NAMES}")

    if args.no_move:
        print("\n(--no-move mode: reading state 5 times, 0.5s apart)")
        for i in range(5):
            q = read_qpos(robot)
            print(f"  t+{i*0.5:.1f}s: {q.round(2)}")
            time.sleep(0.5)
        robot.disconnect()
        return

    step = args.step
    print(f"\n=== Joint-by-joint test (step {step}°, each held 1.5s) ===")
    print("Watch each motor: it should move by the step and come back.")
    print("Press Ctrl+C to abort.\n")

    for j, name in enumerate(MOTOR_NAMES):
        q_now = read_qpos(robot)
        cur = q_now[j]
        print(f"--- {name} [idx {j}] current={cur:.1f}° ---")

        target_plus = np.clip(cur + step, -180, 180)
        action = {f"{m}.pos": float(cur if i != j else target_plus)
                  for i, m in enumerate(MOTOR_NAMES)}
        robot.send_action(action)
        time.sleep(1.5)
        q_after = read_qpos(robot)
        print(f"  cmd +{step:>5.1f}° -> target={target_plus:7.1f}°  read={q_after[j]:7.2f}°  "
              f"delta={(q_after[j]-cur):+6.2f}°")

        target_minus = np.clip(cur - step, -180, 180)
        action = {f"{m}.pos": float(cur if i != j else target_minus)
                  for i, m in enumerate(MOTOR_NAMES)}
        robot.send_action(action)
        time.sleep(1.5)
        q_after2 = read_qpos(robot)
        print(f"  cmd -{step:>5.1f}° -> target={target_minus:7.1f}°  read={q_after2[j]:7.2f}°  "
              f"delta={(q_after2[j]-cur):+6.2f}°")

        action = {f"{m}.pos": float(cur) for m in MOTOR_NAMES}
        robot.send_action(action)
        time.sleep(1.0)

        d1 = q_after[j] - cur
        d2 = q_after2[j] - cur
        if abs(d1) > step * 0.3 and abs(d2) > step * 0.3 and d1 > 0 and d2 < 0:
            print(f"  [OK] motor responds correctly")
        elif abs(d1) < step * 0.1 and abs(d2) < step * 0.1:
            print(f"  [FAIL] motor did NOT move — check wiring/power/ID")
        else:
            print(f"  [CHECK] responds but direction/scale odd (d1={d1:+.1f} d2={d2:+.1f})")

    robot.disconnect()
    print("\nDone.")


if __name__ == '__main__':
    main()
