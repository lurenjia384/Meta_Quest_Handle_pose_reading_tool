#!/usr/bin/env python3
"""Recover both Flexiv arms after an e-stop/fault, then reset them to a shared joint pose."""

from __future__ import annotations

import argparse
import concurrent.futures
import math
import time
from typing import Any, Sequence


DEFAULT_ROBOTS = ["Rizon4-062833", "Rizon4-062084"]

# Shared screw reset target captured from the current slave arm posture on 2026-07-13 12:07:03.
# Slave q rad: [-0.792182, -0.340862, 0.900836, 2.072172, -0.338952, 0.685745, 1.319355]
DEFAULT_RESET_JOINT_POS_RAD = [
    -0.792182,
    -0.340862,
    0.900836,
    2.072172,
    -0.338952,
    0.685745,
    1.319355,
]


def _safe_call(obj: Any, name: str) -> Any:
    method = getattr(obj, name, None)
    if method is None:
        return "unavailable"
    try:
        return method()
    except Exception as exc:  # noqa: BLE001 - diagnostics must not hide the main failure.
        return f"error:{exc}"


def _bool_text(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def robot_diagnostics(robot: Any) -> dict[str, Any]:
    return {
        "mode": _safe_call(robot, "mode"),
        "fault": _safe_call(robot, "fault"),
        "operational": _safe_call(robot, "operational"),
        "op_status": _safe_call(robot, "operational_status"),
        "reduced": _safe_call(robot, "reduced"),
        "recovery": _safe_call(robot, "recovery"),
        "estop_released": _safe_call(robot, "estop_released"),
        "enabling_button_pressed": _safe_call(robot, "enabling_button_pressed"),
        "timeliness_limit_reached": _safe_call(robot, "reached_timeliness_failure_limit"),
    }


def print_diagnostics(sn: str, robot: Any, prefix: str) -> None:
    diag = robot_diagnostics(robot)
    fields = " ".join(f"{key}={_bool_text(value)}" for key, value in diag.items())
    print(f"[{sn}] {prefix}: {fields}")


def wait_condition(
    *,
    sn: str,
    label: str,
    timeout_s: float,
    poll_s: float,
    condition,
    status=None,
) -> None:
    deadline = time.monotonic() + timeout_s
    last_log = 0.0
    while True:
        value = condition()
        if value:
            return
        now = time.monotonic()
        if now > deadline:
            detail = f", status={status()}" if status is not None else ""
            raise TimeoutError(f"{sn} timed out waiting for {label}{detail}")
        if now - last_log >= 1.0:
            detail = f" {status()}" if status is not None else ""
            print(f"[{sn}] waiting for {label}{detail}")
            last_log = now
        time.sleep(poll_s)


def wait_estop_released(robot: Any, sn: str, timeout_s: float) -> None:
    value = _safe_call(robot, "estop_released")
    if value == "unavailable":
        print(f"[{sn}] estop_released diagnostic unavailable; continuing")
        return
    wait_condition(
        sn=sn,
        label="e-stop release",
        timeout_s=timeout_s,
        poll_s=0.1,
        condition=lambda: bool(_safe_call(robot, "estop_released")),
        status=lambda: f"estop_released={_bool_text(_safe_call(robot, 'estop_released'))}",
    )


def clear_fault(robot: Any, sn: str, clear_fault_timeout_s: float) -> None:
    if not bool(_safe_call(robot, "fault")):
        return
    print(f"[{sn}] fault detected, calling ClearFault")
    timeout_arg = max(1, int(math.ceil(clear_fault_timeout_s)))
    try:
        ok = robot.ClearFault(timeout_arg)
    except TypeError:
        ok = robot.ClearFault()
    if not ok:
        raise RuntimeError(f"{sn} fault cannot be cleared")
    time.sleep(0.5)
    if bool(_safe_call(robot, "fault")):
        raise RuntimeError(f"{sn} fault still present after ClearFault")


def enable_and_wait_operational(robot: Any, sn: str, timeout_s: float) -> None:
    print(f"[{sn}] enabling")
    robot.Enable()
    wait_condition(
        sn=sn,
        label="operational",
        timeout_s=timeout_s,
        poll_s=0.1,
        condition=lambda: bool(_safe_call(robot, "operational")),
        status=lambda: (
            f"fault={_bool_text(_safe_call(robot, 'fault'))} "
            f"op_status={_safe_call(robot, 'operational_status')}"
        ),
    )


def recover_robot(robot: Any, sn: str, args: argparse.Namespace) -> None:
    print_diagnostics(sn, robot, "diagnostics before recovery")
    wait_estop_released(robot, sn, args.estop_release_timeout_s)

    if bool(_safe_call(robot, "recovery")):
        if args.run_auto_recovery:
            print(f"[{sn}] recovery=true, running RunAutoRecovery")
            robot.RunAutoRecovery()
            raise RuntimeError(
                f"{sn} automatic recovery finished; Flexiv RDK requires a robot reboot before reset"
            )
        raise RuntimeError(
            f"{sn} is in recovery state. Use --run-auto-recovery only for joint-limit recovery, "
            "then reboot the robot before running reset again."
        )

    clear_fault(robot, sn, args.clear_fault_timeout_s)
    enable_and_wait_operational(robot, sn, args.operational_timeout_s)
    print_diagnostics(sn, robot, "diagnostics after recovery")


def max_abs(values: Sequence[float]) -> float:
    return max((abs(float(v)) for v in values), default=0.0)


def reset_joint_position(robot: Any, sn: str, target_q: list[float], args: argparse.Namespace) -> tuple[float, list[float]]:
    dof = len(target_q)
    zero = [0.0] * dof
    max_vel = [float(args.jointpos_max_vel_rad_s)] * dof
    max_acc = [float(args.jointpos_max_acc_rad_s2)] * dof
    print(f"[{sn}] switching to NRT_JOINT_POSITION")
    robot.SwitchMode(args.flexivrdk.Mode.NRT_JOINT_POSITION)
    print(f"[{sn}] reset target_q_rad={[round(v, 6) for v in target_q]}")
    robot.SendJointPosition(target_q, zero, max_vel, max_acc)

    deadline = time.monotonic() + args.timeout_s
    last_log = 0.0
    while True:
        states = robot.states()
        q = [float(v) for v in states.q]
        dq = [float(v) for v in states.dq]
        max_q_err = max_abs([a - b for a, b in zip(q, target_q)])
        max_dq = max_abs(dq)
        if max_q_err <= args.final_joint_tol_rad and max_dq <= args.final_dq_tol_rad_s:
            break
        now = time.monotonic()
        if now > deadline:
            robot.Stop()
            raise TimeoutError(
                f"{sn} reset timed out, max_q_err_rad={max_q_err:.6f}, "
                f"max_dq_rad_s={max_dq:.6f}"
            )
        if now - last_log >= 1.0:
            print(f"[{sn}] moving, max_q_err_rad={max_q_err:.6f}, max_dq_rad_s={max_dq:.6f}")
            last_log = now
        time.sleep(0.05)

    time.sleep(0.2)
    states = robot.states()
    q = [float(v) for v in states.q]
    pose = [float(v) for v in states.tcp_pose]
    max_q_err = max_abs([a - b for a, b in zip(q, target_q)])
    print(f"[{sn}] reached, max_q_err_rad={max_q_err:.6f}, tcp_pose={[round(v, 6) for v in pose]}")
    robot.Stop()
    print(f"[{sn}] stopped")
    return max_q_err, pose


def run_one(sn: str, target_q: list[float], args: argparse.Namespace) -> tuple[float, list[float]]:
    print(f"[{sn}] connecting")
    robot = args.flexivrdk.Robot(sn)
    try:
        recover_robot(robot, sn, args)
        return reset_joint_position(robot, sn, target_q, args)
    finally:
        try:
            print_diagnostics(sn, robot, "diagnostics final")
        except Exception:  # noqa: BLE001
            pass


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-sn", action="append", default=None, help="Robot SN; repeat for both arms")
    parser.add_argument("--target-joints-rad", type=float, nargs=7, default=DEFAULT_RESET_JOINT_POS_RAD)
    parser.add_argument("--jointpos-max-vel-rad-s", type=float, default=0.45)
    parser.add_argument("--jointpos-max-acc-rad-s2", type=float, default=0.90)
    parser.add_argument("--final-joint-tol-rad", type=float, default=0.002)
    parser.add_argument("--final-dq-tol-rad-s", type=float, default=0.02)
    parser.add_argument("--estop-release-timeout-s", type=float, default=120.0)
    parser.add_argument("--clear-fault-timeout-s", type=float, default=30.0)
    parser.add_argument("--operational-timeout-s", type=float, default=30.0)
    parser.add_argument("--timeout-s", type=float, default=90.0)
    parser.add_argument("--serial", action="store_true", help="Recover/reset arms one by one instead of concurrently")
    parser.add_argument(
        "--run-auto-recovery",
        action="store_true",
        help="Run Flexiv automatic recovery if recovery=true; robot reboot is still required afterward",
    )
    parser.add_argument(
        "--gripper",
        action="store_true",
        help="Open the Robotiq gripper after reset (default: do NOT touch the gripper)",
    )
    parser.add_argument(
        "--gripper-port",
        default="/dev/ttyACM0",
        help="Robotiq gripper serial port (default /dev/ttyACM0)",
    )
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    positive = {
        "jointpos_max_vel_rad_s": args.jointpos_max_vel_rad_s,
        "jointpos_max_acc_rad_s2": args.jointpos_max_acc_rad_s2,
        "final_joint_tol_rad": args.final_joint_tol_rad,
        "estop_release_timeout_s": args.estop_release_timeout_s,
        "clear_fault_timeout_s": args.clear_fault_timeout_s,
        "operational_timeout_s": args.operational_timeout_s,
        "timeout_s": args.timeout_s,
    }
    for name, value in positive.items():
        if value <= 0.0:
            raise ValueError(f"{name} must be positive")
    if args.final_dq_tol_rad_s < 0.0:
        raise ValueError("final_dq_tol_rad_s must be non-negative")


def main() -> int:
    args = parse_args()
    validate_args(args)

    import flexivrdk  # Imported after --help parsing so the CLI remains inspectable.

    args.flexivrdk = flexivrdk
    robots = args.robot_sn if args.robot_sn else DEFAULT_ROBOTS
    target_q = [float(v) for v in args.target_joints_rad]

    print(f"robots={robots}")
    print(f"target_joints_rad={[round(v, 6) for v in target_q]}")
    print(
        "joint_position_limits="
        f"max_vel={args.jointpos_max_vel_rad_s:.3f}rad/s "
        f"max_acc={args.jointpos_max_acc_rad_s2:.3f}rad/s^2"
    )
    print("parallel_reset=%s" % ("false" if args.serial or len(robots) <= 1 else "true"))

    results: dict[str, tuple[float, list[float]]] = {}
    errors: dict[str, str] = {}

    def run_one_tolerant(sn: str) -> None:
        try:
            results[sn] = run_one(sn, target_q, args)
        except Exception as exc:  # noqa: BLE001 - one failure must not kill the batch
            errors[sn] = str(exc)
            print(f"[{sn}] FAILED: {exc}")

    if args.serial or len(robots) <= 1:
        for sn in robots:
            run_one_tolerant(sn)
    else:
        with concurrent.futures.ThreadPoolExecutor(max_workers=len(robots)) as executor:
            futures = {executor.submit(run_one_tolerant, sn): sn for sn in robots}
            for future in concurrent.futures.as_completed(futures):
                future.result()
        results = {sn: results[sn] for sn in robots if sn in results}

    print("summary:")
    poses = []
    for sn, (max_q_err, pose) in results.items():
        poses.append((sn, pose))
        print(f"  {sn}: max_q_err_rad={max_q_err:.6f}")
    if errors:
        print("failures:")
        for sn, err in errors.items():
            print(f"  {sn}: {err}")
    if len(poses) >= 2:
        a_sn, a_pose = poses[0]
        b_sn, b_pose = poses[1]
        pos_diff_mm = math.sqrt(sum((a - b) ** 2 for a, b in zip(a_pose[:3], b_pose[:3]))) * 1000.0
        print(f"  {a_sn} vs {b_sn}: tcp_pos_diff_mm={pos_diff_mm:.3f}")

    # Optional: open the Robotiq gripper after reset
    if args.gripper:
        try:
            from gripper_modbus import RobotiqGripper
            gripper = RobotiqGripper(args.gripper_port)
            gripper.open()
            time.sleep(1.0)
            print("  Robotiq gripper OPENED")
            gripper.close_port()
        except Exception as exc:
            print("  WARNING: could not open gripper: {}".format(exc))

    return 0 if not errors else 1


if __name__ == "__main__":
    raise SystemExit(main())
