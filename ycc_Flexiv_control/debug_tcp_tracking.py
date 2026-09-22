#!/usr/bin/env python3
"""
Debug: does the Flexiv arm accurately track TCP targets?

Streams a series of small TCP targets (within the safe 20cm envelope) and
reads back the real TCP from states() each frame to measure:
  - tracking error (target vs real)
  - closed-loop latency (how many frames behind the arm is)
  - smoothness (per-frame real velocity jitter)

Usage (RoboTwin env):
    conda activate RoboTwin
    python debug_tcp_tracking.py [--duration 8] [--amplitude 0.05]
"""

import argparse
import sys
import time

import numpy as np
import flexivrdk


def wait_until_operational(robot, timeout_s=15.0):
    deadline = time.time() + timeout_s
    while not robot.operational():
        if time.time() > deadline:
            raise TimeoutError("robot not operational")
        time.sleep(0.2)


def read_tcp(robot):
    s = robot.states()
    return np.asarray(s.tcp_pose, dtype=np.float64).reshape(7)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sn", default="Rizon4-062084")
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--amplitude", type=float, default=0.04)
    parser.add_argument("--freq", type=float, default=0.3)
    args = parser.parse_args()

    print("connecting...")
    robot = flexivrdk.Robot(args.sn)
    if robot.fault():
        robot.ClearFault()
        time.sleep(0.5)
    if not robot.estop_released():
        print("E-stop NOT released")
        return 1
    robot.Enable()
    wait_until_operational(robot)
    print("operational")

    start_tcp = read_tcp(robot)
    print("start tcp:", np.array2string(start_tcp, precision=4, floatmode="fixed"))

    robot.SwitchMode(flexivrdk.Mode.NRT_CARTESIAN_MOTION_FORCE)
    time.sleep(0.1)
    print("switched to NRT_CARTESIAN_MOTION_FORCE")

    # sine trajectory around start position
    FPS = 50.0
    n = int(args.duration * FPS)
    targets = []
    for i in range(n):
        t = i / FPS
        target = start_tcp.copy()
        target[0] += args.amplitude * np.sin(2 * np.pi * args.freq * t)
        target[1] += args.amplitude * 0.5 * np.sin(2 * np.pi * args.freq * t + 1.0)
        target[2] += args.amplitude * 0.3 * np.sin(2 * np.pi * args.freq * t + 2.0)
        targets.append(target)

    print("streaming sine targets, reading real TCP...\n")
    print("  frame  tgt_x    real_x   err_mm  lag_frames  real_v(m/s)")
    max_err = 0.0
    max_lag = 0
    frames_behind = []
    real_prev = start_tcp[:3]
    t_prev = time.time()

    loop_start = time.time()
    for i, target in enumerate(targets):
        t0 = time.time()
        robot.SendCartesianMotionForce(target.astype(float).tolist(), [0.0] * 6)
        real = read_tcp(robot)
        dt = time.time() - t0

        err = np.linalg.norm(real[:3] - target[:3])
        max_err = max(max_err, err)

        # velocity of real tcp
        v_real = np.linalg.norm(real[:3] - real_prev) / (dt + 1e-9)
        real_prev = real[:3]

        # crude lag estimate: compare real position to target history
        # (find which past target matches current real position best)
        if i >= 5:
            best_lag = 0
            best_d = 1e9
            for lag in range(0, min(i, 30)):
                d = np.linalg.norm(real[:3] - targets[i - lag][:3])
                if d < best_d:
                    best_d = d
                    best_lag = lag
            max_lag = max(max_lag, best_lag)
            frames_behind.append(best_lag)

        if i % 10 == 0:
            print(f"  {i:4d}  {target[0]:.4f}  {real[0]:.4f}  {err*1000:5.1f}  "
                  f"{best_lag if i>=5 else 0:4d}  {v_real:7.3f}")

        # maintain FPS
        elapsed = time.time() - loop_start - (i + 1) / FPS
        if elapsed < 0:
            time.sleep(-elapsed)

    robot.Stop()
    print("\n=== Results ===")
    print(f"  max tracking error: {max_err*1000:.1f} mm")
    if frames_behind:
        avg_lag = np.mean(frames_behind)
        print(f"  avg lag: {avg_lag:.1f} frames ({(avg_lag/FPS)*1000:.0f} ms @ {FPS:.0f}Hz)")
        print(f"  max lag: {max_lag} frames")
    print(f"  loop send+read latency: {dt*1000:.2f} ms/frame")
    robot.Stop()
    print("done")


if __name__ == "__main__":
    main()
