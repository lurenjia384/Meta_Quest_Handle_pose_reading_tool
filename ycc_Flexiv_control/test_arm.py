import argparse
import math
import sys
import time

import numpy as np
import flexivrdk


def wait_until_operational(robot, timeout_s: float = 15.0) -> None:
    deadline = time.time() + timeout_s
    while not robot.operational():
        if time.time() > deadline:
            raise TimeoutError(
                f"robot not operational after {timeout_s}s "
                f"(operational_status={int(robot.operational_status())})"
            )
        time.sleep(0.2)


def read_state(robot):
    s = robot.states()
    q = np.asarray(s.q, dtype=np.float64).reshape(-1)
    tcp = np.asarray(s.tcp_pose, dtype=np.float64).reshape(7)
    return {
        "q_rad": q,
        "tcp_pose_xyz_quat_wxyz": tcp,
        "mode": int(robot.mode()),
        "operational": bool(robot.operational()),
        "op_status": str(robot.operational_status()),
        "estop_released": bool(robot.estop_released()),
        "fault": int(robot.fault()),
    }


def fmt(arr, precision: int = 5) -> str:
    return np.array2string(np.asarray(arr), precision=precision, floatmode="fixed")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sn", default="Rizon4-062084",
                        help="robot serial number (default: Rizon4-062084)")
    parser.add_argument("--dq-rad", type=float, default=0.001,
                        help="tiny joint-1 motion in rad (default 0.001, ~0.1% of range)")
    parser.add_argument("--max-vel", type=float, default=0.02,
                        help="joint velocity limit rad/s (default 0.02)")
    parser.add_argument("--max-acc", type=float, default=0.05,
                        help="joint acceleration limit rad/s^2 (default 0.05)")
    parser.add_argument("--timeout-s", type=float, default=20.0,
                        help="motion settle timeout (default 20)")
    parser.add_argument("--no-return", action="store_true",
                        help="do not command back to start on failure/exit")
    args = parser.parse_args()

    robot = flexivrdk.Robot(args.sn)
    print(f"[1/5] connected to {args.sn} (fault={int(robot.fault())})", flush=True)

    if robot.fault():
        print("  robot has a fault; clearing...", flush=True)
        if not robot.ClearFault():
            print("  ERROR: failed to clear fault", flush=True)
            return 1
        time.sleep(0.5)

    if not robot.estop_released():
        print("  WARNING: emergency stop NOT released; robot cannot move. "
              "Release the physical E-stop before retrying.", flush=True)
        return 3

    robot.Enable()
    wait_until_operational(robot, timeout_s=15.0)
    print("  robot enabled + operational", flush=True)

    print("[2/5] reading initial state...", flush=True)
    before = read_state(robot)
    print(f"  mode={before['mode']} operational={before['operational']} "
          f"op_status={before['op_status']} fault={before['fault']} "
          f"estop_released={before['estop_released']}", flush=True)
    print(f"  joints(q)   = {fmt(before['q_rad'])}", flush=True)
    print(f"  tcp_pose    = {fmt(before['tcp_pose_xyz_quat_wxyz'])}", flush=True)
    q0 = before["q_rad"].copy()
    target = q0.copy()
    dq = min(abs(args.dq_rad), 0.02)
    if q0[0] + dq > math.pi * 0.98:  # stay well inside joint limit
        dq = -dq
    target[0] += dq
    print(f"[3/5] commanding tiny joint-1 motion: dq={dq:.5f} rad "
          f"({abs(dq) / math.pi * 100:.3f}% of +/-pi range)", flush=True)

    try:
        robot.SwitchMode(flexivrdk.Mode.NRT_JOINT_POSITION)
        time.sleep(0.2)
        dof = len(target)
        zero_vel = [0.0] * dof
        max_vel = [float(args.max_vel)] * dof
        max_acc = [float(args.max_acc)] * dof
        robot.SendJointPosition(target.astype(float).tolist(), zero_vel, max_vel, max_acc)
        print("  target joint positions sent", flush=True)

        deadline = time.time() + args.timeout_s
        while True:
            s = robot.states()
            q_cur = np.asarray(s.q, dtype=np.float64).reshape(-1)
            err = abs(q_cur[0] - target[0])
            if err < 5e-4:
                break
            if time.time() > deadline:
                print(f"  WARNING: settle timeout; err={err:.5f} rad", flush=True)
                break
            time.sleep(0.2)
    except Exception as exc:
        print(f"  ERROR during motion: {exc}", flush=True)
        if not args.no_return:
            try:
                print("  commanding back to start posture (best effort)...", flush=True)
                robot.SwitchMode(flexivrdk.Mode.NRT_JOINT_POSITION)
                dof = len(q0)
                zero_vel = [0.0] * dof
                max_vel = [float(args.max_vel)] * dof
                max_acc = [float(args.max_acc)] * dof
                robot.SendJointPosition(q0.astype(float).tolist(), zero_vel, max_vel, max_acc)
            except Exception:
                pass
        return 1

    print("[4/5] reading state after motion...", flush=True)
    after = read_state(robot)
    print(f"  joints(q)   = {fmt(after['q_rad'])}", flush=True)
    print(f"  tcp_pose    = {fmt(after['tcp_pose_xyz_quat_wxyz'])}", flush=True)

    delta = after["q_rad"] - before["q_rad"]
    print(f"  joint delta = {fmt(delta)}", flush=True)
    moved = np.linalg.norm(delta) > 1e-4
    print(f"  RESULT: {'MOTION CONFIRMED' if moved else 'NO VISIBLE MOTION'} "
          f"(|dq|={np.linalg.norm(delta):.5f})", flush=True)

    if not args.no_return:
        print("[5/5] returning to start posture...", flush=True)
        robot.SwitchMode(flexivrdk.Mode.NRT_JOINT_POSITION)
        dof = len(q0)
        zero_vel = [0.0] * dof
        max_vel = [float(args.max_vel)] * dof
        max_acc = [float(args.max_acc)] * dof
        robot.SendJointPosition(q0.astype(float).tolist(), zero_vel, max_vel, max_acc)
        deadline = time.time() + args.timeout_s
        while True:
            s = robot.states()
            q_cur = np.asarray(s.q, dtype=np.float64).reshape(-1)
            if np.linalg.norm(q_cur - q0) < 5e-4:
                break
            if time.time() > deadline:
                print("  WARNING: return-to-start timeout", flush=True)
                break
            time.sleep(0.2)
        print("  returned to start", flush=True)

    return 0 if moved else 2


if __name__ == "__main__":
    sys.exit(main())
