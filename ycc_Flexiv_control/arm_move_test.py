import argparse
import math
import sys
import time

import numpy as np
import flexivrdk

DOF = 7
J7_INDEX = 6


def wait_until_operational(robot, timeout_s: float = 15.0) -> None:
    deadline = time.time() + timeout_s
    while not robot.operational():
        if time.time() > deadline:
            raise TimeoutError(
                "robot not operational after {}s (op_status={})".format(
                    timeout_s, robot.operational_status()
                )
            )
        time.sleep(0.2)


def read_q(robot) -> np.ndarray:
    s = robot.states()
    return np.asarray(s.q, dtype=np.float64).reshape(-1)


def read_tcp(robot) -> np.ndarray:
    s = robot.states()
    return np.asarray(s.tcp_pose, dtype=np.float64).reshape(7)


def fmt(arr, precision: int = 5) -> str:
    return np.array2string(np.asarray(arr), precision=precision, floatmode="fixed")


def send_joint_target(robot, target_q: np.ndarray, max_vel: float, max_acc: float) -> None:
    zero_vel = [0.0] * DOF
    max_vel_l = [float(max_vel)] * DOF
    max_acc_l = [float(max_acc)] * DOF
    robot.SwitchMode(flexivrdk.Mode.NRT_JOINT_POSITION)
    time.sleep(0.1)
    robot.SendJointPosition(target_q.astype(float).tolist(), zero_vel, max_vel_l, max_acc_l)


def wait_joint(robot, target_q: np.ndarray, tol_rad: float, timeout_s: float) -> bool:
    deadline = time.time() + timeout_s
    while True:
        q = read_q(robot)
        if float(np.max(np.abs(q - target_q))) <= tol_rad:
            return True
        if time.time() > deadline:
            print("  WARNING: joint wait timeout; max_err={:.5f}".format(
                float(np.max(np.abs(q - target_q)))), flush=True)
            return False
        time.sleep(0.2)


def send_tcp_target(robot, target_pose: np.ndarray) -> None:
    robot.SwitchMode(flexivrdk.Mode.NRT_CARTESIAN_MOTION_FORCE)
    time.sleep(0.1)
    command_wrench = [0.0] * 6
    robot.SendCartesianMotionForce(target_pose.astype(float).tolist(), command_wrench)


def wait_tcp(robot, target_pose: np.ndarray, tol_m: float, timeout_s: float) -> bool:
    deadline = time.time() + timeout_s
    while True:
        pose = read_tcp(robot)
        err = float(np.linalg.norm(pose[:3] - target_pose[:3]))
        if err <= tol_m:
            return True
        if time.time() > deadline:
            print("  WARNING: TCP wait timeout; pos_err={:.5f} m".format(err), flush=True)
            return False
        time.sleep(0.2)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sn", default="Rizon4-062084")
    parser.add_argument("--j7-frac", type=float, default=0.30,
                        help="fraction of J7 range to rotate (default 0.30)")
    parser.add_argument("--tcp-dz-m", type=float, default=0.01,
                        help="TCP +Z displacement in meters (default 0.01)")
    parser.add_argument("--max-vel", type=float, default=0.10,
                        help="joint velocity limit rad/s (default 0.10)")
    parser.add_argument("--max-acc", type=float, default=0.20,
                        help="joint acceleration limit rad/s^2 (default 0.20)")
    parser.add_argument("--cart-vel", type=float, default=0.02,
                        help="cartesian max linear velocity m/s (default 0.02)")
    parser.add_argument("--cart-acc", type=float, default=0.05,
                        help="cartesian max linear acceleration m/s^2 (default 0.05)")
    parser.add_argument("--timeout-s", type=float, default=30.0)
    parser.add_argument("--no-return", action="store_true",
                        help="do not return to start posture")
    args = parser.parse_args()

    robot = flexivrdk.Robot(args.sn)
    print("[1/5] connected, fault={}".format(int(robot.fault())), flush=True)
    if robot.fault():
        if not robot.ClearFault():
            print("  ERROR: cannot clear fault", flush=True)
            return 1
        time.sleep(0.5)
    if not robot.estop_released():
        print("  WARNING: E-stop NOT released; aborting", flush=True)
        return 3
    robot.Enable()
    wait_until_operational(robot)
    print("  robot enabled + operational", flush=True)

    q0 = read_q(robot)
    tcp0 = read_tcp(robot)
    print("[2/5] start q = {}".format(fmt(q0)), flush=True)
    print("      start tcp = {}".format(fmt(tcp0)), flush=True)

    info = robot.info()
    q_min = np.asarray(info.q_min, dtype=np.float64).reshape(-1)
    q_max = np.asarray(info.q_max, dtype=np.float64).reshape(-1)
    print("      J7 range: [{:.4f}, {:.4f}] rad".format(q_min[J7_INDEX], q_max[J7_INDEX]), flush=True)

    margin = math.radians(5.0)
    safe_min = float(q_min[J7_INDEX] + margin)
    safe_max = float(q_max[J7_INDEX] - margin)
    delta_j7 = float(args.j7_frac) * (float(q_max[J7_INDEX]) - float(q_min[J7_INDEX]))
    target_q = q0.copy()
    target_q[J7_INDEX] = float(np.clip(q0[J7_INDEX] + delta_j7, safe_min, safe_max))
    actual_delta = target_q[J7_INDEX] - q0[J7_INDEX]
    if abs(actual_delta) < 1e-6:
        print("  ERROR: J7 has no room to rotate (at soft limit)", flush=True)
        return 2
    print("[3/5] J7 rotation: requested_delta={:.4f} rad clipped_to={:.4f} rad ({:.1f}% of range)".format(
        delta_j7, actual_delta, actual_delta / (q_max[J7_INDEX] - q_min[J7_INDEX]) * 100.0), flush=True)

    send_joint_target(robot, target_q, args.max_vel, args.max_acc)
    ok = wait_joint(robot, target_q, tol_rad=0.005, timeout_s=args.timeout_s)
    print("  J7 move done, q = {}".format(fmt(read_q(robot))), flush=True)

    tcp_after_j7 = read_tcp(robot)
    target_pose = tcp_after_j7.copy()
    target_pose[2] += float(args.tcp_dz_m)
    print("[4/5] TCP +Z move: dz={:.3f} m, target tcp = {}".format(
        args.tcp_dz_m, fmt(target_pose)), flush=True)
    send_tcp_target(robot, target_pose)
    ok2 = wait_tcp(robot, target_pose, tol_m=0.002, timeout_s=args.timeout_s)
    print("  TCP move done, tcp = {}".format(fmt(read_tcp(robot))), flush=True)

    success = ok and ok2
    print("  RESULT: {}".format("BOTH MOTIONS OK" if success else "PARTIAL / TIMEOUT"), flush=True)

    if not args.no_return:
        print("[5/5] returning to start posture...", flush=True)
        send_joint_target(robot, q0, args.max_vel, args.max_acc)
        wait_joint(robot, q0, tol_rad=0.005, timeout_s=args.timeout_s)
        send_tcp_target(robot, tcp0)
        wait_tcp(robot, tcp0, tol_m=0.002, timeout_s=args.timeout_s)
        print("  returned to start, q = {}".format(fmt(read_q(robot))), flush=True)
        print("  returned tcp = {}".format(fmt(read_tcp(robot))), flush=True)

    return 0 if success else 4


if __name__ == "__main__":
    sys.exit(main())
