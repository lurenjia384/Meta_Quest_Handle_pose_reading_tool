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
SHORT = {
    "pan": 0, "lift": 1, "elbow": 2, "wflex": 3, "wroll": 4, "grip": 5,
}


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


def send_all(robot, q, target_idx, target_val):
    action = {f"{m}.pos": float(target_val if i == target_idx else q[i])
              for i, m in enumerate(MOTOR_NAMES)}
    robot.send_action(action)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='/dev/ttyACM0')
    parser.add_argument('--step', type=float, default=3.0,
                        help='test step size in degrees (default 3, keep small!)')
    parser.add_argument('--joint', default='all',
                        help='pan|lift|elbow|wflex|wroll|grip|all')
    args = parser.parse_args()

    step = abs(args.step)
    if step > 10:
        print(f"WARNING: step {step}° is large. Reduce to 3-5° for safety.")

    if args.joint == 'all':
        test_indices = list(range(6))
    else:
        test_indices = [SHORT[args.joint]]

    robot = connect_robot(args.port)
    q0 = read_qpos(robot)
    print(f"\nInitial qpos: {q0.round(2)}°")
    print(f"Testing joints: {[MOTOR_NAMES[i] for i in test_indices]}, step={step}°\n")

    results = {}
    try:
        for idx in test_indices:
            name = MOTOR_NAMES[idx]
            q_start = read_qpos(robot)
            base = q_start[idx]
            print(f"--- {name} [idx {idx}] base={base:.1f}° ---")

            send_all(robot, q_start, idx, base + step)
            time.sleep(1.2)
            q1 = read_qpos(robot)
            d1 = q1[idx] - base

            send_all(robot, q_start, idx, base - step)
            time.sleep(1.2)
            q2 = read_qpos(robot)
            d2 = q2[idx] - base

            send_all(robot, q_start, idx, base)
            time.sleep(1.0)

            print(f"  cmd +{step}° -> read {d1:+.2f}°  |  cmd -{step}° -> read {d2:+.2f}°")
            # direction match: +cmd should give +read, -cmd should give -read
            if d1 > 0.3 * step and d2 < -0.3 * step:
                verdict = "NORMAL (no flip needed)"
                flip = False
            elif d1 < -0.3 * step and d2 > 0.3 * step:
                verdict = "REVERSED (need SIGN_FLIP)"
                flip = True
            else:
                verdict = f"UNCLEAR (d1={d1:+.1f} d2={d2:+.1f}) — motor may be blocked"
                flip = None
            print(f"  -> {verdict}")
            results[name] = flip

    except KeyboardInterrupt:
        print("\nInterrupted — returning to start pose.")
    finally:
        try:
            send_all(robot, read_qpos(robot), -1, 0.0)  # no-op-ish
        except Exception:
            pass
        robot.disconnect()

    print("\n=== Direction summary ===")
    print("Add to real_teleop.py SIGN_FLIP = {")
    for name, flip in results.items():
        print(f"    {name!r}: {bool(flip) if flip is not None else False},")
    print("}")


if __name__ == '__main__':
    main()
