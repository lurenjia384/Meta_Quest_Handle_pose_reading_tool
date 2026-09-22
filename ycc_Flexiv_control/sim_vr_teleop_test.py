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
    return np.asarray(robot.states().tcp_pose, dtype=np.float64).reshape(7)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sn", default="Rizon4-062084")
    parser.add_argument("--pattern", default="smooth",
                        choices=["smooth", "zigzag", "noisy", "step"])
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--fps", type=float, default=60.0)
    parser.add_argument("--scale", type=float, default=0.3)
    parser.add_argument("--max-step-mm", type=float, default=2.0,
                        help="per-frame position step cap in mm")
    args = parser.parse_args()

    FPS = args.fps
    max_step_m = args.max_step_mm / 1000.0
    max_rot_step = 0.003
    max_step = max_step_m
    pos_scale = args.scale
    rot_scale = args.scale

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
    print(f"pattern={args.pattern}  fps={FPS}  max_step={max_step*1000:.1f}mm/frame")

    vr_ref = np.eye(4)
    vr_ref[:3, 3] = [0.15, -0.30, -0.25]

    def gen_handle(t, i):
        h = vr_ref.copy()
        if args.pattern == "smooth":
            h[:3, 3] += [
                0.06 * np.sin(2 * np.pi * 0.35 * t),
                0.04 * np.sin(2 * np.pi * 0.28 * t + 1.0),
                0.05 * np.sin(2 * np.pi * 0.22 * t + 2.0),
            ]
        elif args.pattern == "zigzag":
            phase = (t % 0.8) / 0.8
            x = 0.06 * (2.0 * abs(2.0 * phase - 1.0) - 1.0)
            h[:3, 3] += [x, 0.02 * np.sin(2 * np.pi * 0.5 * t), 0.0]
        elif args.pattern == "noisy":
            h[:3, 3] += [
                0.06 * np.sin(2 * np.pi * 0.35 * t) + 0.004 * np.sin(2 * np.pi * 7 * t),
                0.04 * np.sin(2 * np.pi * 0.28 * t + 1.0),
                0.05 * np.sin(2 * np.pi * 0.22 * t + 2.0),
            ]
        elif args.pattern == "step":
            seg = int(t / 1.5) % 2
            x = 0.02 if seg == 0 else 0.0
            h[:3, 3] += [x, 0.0, 0.0]
        return h

    cur_pose = start_tcp.copy()
    vr_prev = None
    motion_history = []
    real_history = []
    t0 = time.time()

    n = int(args.duration * FPS)
    for i in range(n):
        t = i / FPS
        vr_cur = gen_handle(t, i)
        if vr_prev is None:
            motion = cur_pose.copy()
        else:
            motion = vt_map(vr_cur, vr_prev, cur_pose, pos_scale, rot_scale)
            motion = clamp(cur_pose, motion, max_step, max_rot_step)
            cur_pose = motion.copy()
        vr_prev = vr_cur.copy()

        robot.SendCartesianMotionForce(cur_pose.astype(float).tolist(), [0.0] * 6)
        real = read_tcp(robot)
        motion_history.append(cur_pose[:3].copy())
        real_history.append(real[:3].copy())

        target_t = t0 + (i + 1) / FPS
        sleep = target_t - time.time()
        if sleep > 0:
            time.sleep(sleep)

    robot.Stop()
    mh = np.array(motion_history)
    rh = np.array(real_history)
    err = np.linalg.norm(rh - mh, axis=1)
    vel_real = np.linalg.norm(np.diff(rh, axis=0), axis=1)
    vel_tgt = np.linalg.norm(np.diff(mh, axis=0), axis=1)

    print("\n=== Analysis ===")
    print(f"  max tracking error : {err.max()*1000:.1f} mm")
    print(f"  mean tracking err  : {err.mean()*1000:.1f} mm")
    print(f"  real per-frame disp: mean {vel_real.mean()*1000:.2f} mm, "
          f"max {vel_real.max()*1000:.2f} mm, std {vel_real.std()*1000:.2f} mm")
    print(f"  target per-frame   : mean {vel_tgt.mean()*1000:.2f} mm, "
          f"max {vel_tgt.max()*1000:.2f} mm")
    if vel_tgt.max() > 1e-6:
        jitter = vel_real.std() / (vel_tgt.std() + 1e-9)
        print(f"  jitter ratio (real/target velocity std): {jitter:.2f} "
              f"({'smooth' if jitter < 2 else 'JITTERY'})")
    print("done")


def vt_map(vr_cur, vr_prev, cur_tcp, pos_scale, rot_scale, axis_matrix=None):
    axis_matrix = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])
    T_rel = np.linalg.inv(vr_prev) @ vr_cur
    dp = axis_matrix @ T_rel[:3, 3] * pos_scale
    dR = axis_matrix @ T_rel[:3, :3] @ axis_matrix.T
    rv = matrix_to_rotvec(dR) * rot_scale
    dR_new = rotvec_to_matrix(rv)
    pose = np.empty(7)
    pose[:3] = cur_tcp[:3] + dp
    pose[3:7] = mat_to_quat(dR_new @ quat_to_mat(cur_tcp[3:7]))
    return pose


def clamp(cur, tgt, max_step, max_rot):
    dp = tgt[:3] - cur[:3]
    d = np.linalg.norm(dp)
    if d > max_step:
        dp = dp * (max_step / d)
    Rc = quat_to_mat(cur[3:7]); Rt = quat_to_mat(tgt[3:7])
    rv = matrix_to_rotvec(Rt @ Rc.T)
    ang = np.linalg.norm(rv)
    if ang > 1e-9 and ang > max_rot:
        rv = rv * (max_rot / ang)
    R_new = rotvec_to_matrix(rv) @ Rc
    pose = np.empty(7)
    pose[:3] = cur[:3] + dp
    pose[3:7] = mat_to_quat(R_new)
    return pose


def quat_to_mat(q):
    q = q / np.linalg.norm(q)
    qw, qx, qy, qz = q
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
        [2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx)],
        [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy)],
    ])


def mat_to_quat(R):
    tr = np.trace(R)
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    else:
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            qw = (R[2, 1] - R[1, 2]) / S
            qx = 0.25 * S
            qy = (R[0, 1] + R[1, 0]) / S
            qz = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            qw = (R[0, 2] - R[2, 0]) / S
            qx = (R[0, 1] + R[1, 0]) / S
            qy = 0.25 * S
            qz = (R[1, 2] + R[2, 1]) / S
        else:
            S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            qw = (R[1, 0] - R[0, 1]) / S
            qx = (R[0, 2] + R[2, 0]) / S
            qy = (R[1, 2] + R[2, 1]) / S
            qz = 0.25 * S
    q = np.array([qw, qx, qy, qz])
    return q / np.linalg.norm(q)


def matrix_to_rotvec(R):
    theta = np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    if theta < 1e-9:
        return np.zeros(3)
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * np.sin(theta))
    return axis * theta


def rotvec_to_matrix(rv):
    theta = np.linalg.norm(rv)
    if theta < 1e-9:
        return np.eye(3)
    a = rv / theta
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)


if __name__ == "__main__":
    main()
