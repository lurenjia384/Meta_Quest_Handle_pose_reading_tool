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
HOME_DEG = np.array([0.0, -20.0, 60.0, -30.0, 0.0, 0.0])
MAX_JOINT_DEG = np.array([110.0, 100.0, 96.8, 95.0, 162.8, 100.0])


def connect_robot(port):
    cfg = SO101FollowerConfig(
        port=port,
        id="my_awesome_follower_arm",
        use_degrees=True,
        cameras={},
    )
    robot = SO101Follower(cfg)
    robot.connect(calibrate=False)
    return robot


def read_qpos(robot):
    obs = robot.get_observation()
    return np.array([obs[f"{m}.pos"] for m in MOTOR_NAMES], dtype=np.float64)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='/dev/ttyACM0')
    parser.add_argument('--dry-run', action='store_true',
                        help='print plan without sending actions')
    parser.add_argument('--speed', type=float, default=1.0,
                        help='max per-iteration joint step in deg (default 1.0)')
    args = parser.parse_args()

    robot = connect_robot(args.port)
    q0 = read_qpos(robot)
    print(f"Current: {q0.round(2)}°")
    print(f"Home:    {HOME_DEG.round(1)}°")

    diff = HOME_DEG - q0
    print(f"Diff:    {diff.round(2)}°")
    print(f"Max step/iter: {args.speed}°")

    if args.dry_run:
        print("\n(dry-run) Would interpolate:")
        steps = int(np.abs(diff).max() / args.speed) + 1
        print(f"  {steps} iterations, ~{steps*0.05:.1f}s")
        robot.disconnect()
        return

    target = np.clip(HOME_DEG, -MAX_JOINT_DEG, MAX_JOINT_DEG)

    print("\nMoving to home... (Ctrl+C to abort)")
    q = q0.copy()
    try:
        while True:
            delta = target - q
            step = np.clip(delta, -args.speed, args.speed)
            if np.abs(delta).max() < 0.1:
                break
            q = q + step
            action = {f"{m}.pos": float(q[i]) for i, m in enumerate(MOTOR_NAMES)}
            robot.send_action(action)
            time.sleep(0.05)

        action = {f"{m}.pos": float(target[i]) for i, m in enumerate(MOTOR_NAMES)}
        robot.send_action(action)
        time.sleep(1.0)

        q_final = read_qpos(robot)
        print(f"Reached: {q_final.round(2)}°")
        err = np.abs(q_final - target).max()
        print(f"Max joint error: {err:.2f}°")
    except KeyboardInterrupt:
        print("\nAborted — arm left at current pose.")
    finally:
        robot.disconnect()
        print("Done.")


if __name__ == '__main__':
    main()
