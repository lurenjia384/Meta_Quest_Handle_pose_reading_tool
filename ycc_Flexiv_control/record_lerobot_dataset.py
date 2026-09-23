import argparse
import csv
import signal
import sys
import threading
import time
from pathlib import Path

import numpy as np
import flexivrdk

import vr_teleop as vt
from dual_camera import DualCamera
from gripper_modbus import RobotiqGripper


def read_tcp(robot):
    s = robot.states()
    return np.asarray(s.tcp_pose, dtype=np.float64).reshape(7)


def read_gripper(gripper):
    if gripper is None:
        return -1.0
    try:
        st = gripper.states()
        pos = st.get("position", -1.0) if isinstance(st, dict) else -1.0
        return float(pos)
    except Exception:
        return -1.0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sn", default="Rizon4-062084")
    parser.add_argument("--scale", type=float, default=0.3)
    parser.add_argument("--max-speed", type=float, default=0.8)
    parser.add_argument("--no-rot-scale", action="store_true", default=True)
    parser.add_argument("--gripper", action="store_true")
    parser.add_argument("--camera-devices", default="4,10",
                        help="comma-separated video device indices "
                             "(default 4,10 = D435 color + D435i color)")
    parser.add_argument("--session-dir", default="recordings/session1",
                        help="directory to write raw recording files")
    parser.add_argument("--fps", type=float, default=15.0,
                        help="record FPS (default 15; two RealSense cameras "
                             "share USB bandwidth ~15fps at 640x480)")
    parser.add_argument("--task", default="teleop_task",
                        help="task label for the episode")
    args = parser.parse_args()

    # ---- robot ----
    print("connecting robot...")
    robot = flexivrdk.Robot(args.sn)
    if robot.fault():
        robot.ClearFault()
        time.sleep(0.5)
    if not robot.estop_released():
        print("E-stop NOT released")
        return 1
    robot.Enable()
    deadline = time.time() + 15.0
    while not robot.operational():
        if time.time() > deadline:
            raise TimeoutError("robot not operational")
        time.sleep(0.2)
    cur_pose = read_tcp(robot)
    print("start tcp =", np.array2string(cur_pose, precision=4, floatmode="fixed"))

    gripper = None
    if args.gripper:
        try:
            gripper = RobotiqGripper("/dev/ttyACM0")
            gripper.open()
            print("Robotiq gripper ready")
        except Exception as exc:
            print("WARNING: gripper unavailable:", exc)
            gripper = None

    devs = tuple(int(x) for x in args.camera_devices.split(","))
    print(f"starting cameras on {devs}...")
    cam = DualCamera(dev_indices=devs)
    cam.start()

    vr = vt.VRPoseReader()
    vr.start()
    print("waiting for VR data...")
    while True:
        lp, rp, btns = vr.get_state()
        if rp is not None:
            break
        time.sleep(0.5)
    print("VR ready.")

    robot.SwitchMode(flexivrdk.Mode.NRT_CARTESIAN_MOTION_FORCE)
    time.sleep(0.1)
    print("switched to NRT_CARTESIAN_MOTION_FORCE")

    session = Path(args.session_dir)
    session.mkdir(parents=True, exist_ok=True)
    ep_idx = 0
    print(f"session dir: {session}")

    pos_scale = args.scale
    rot_scale = 1.0 if args.no_rot_scale else 0.3
    axis_matrix = vt.VR_TO_ROBOT
    governor = vt.ReferenceGovernor()
    vr_ref = None
    tcp_ref = None
    prev_loop_t = None
    prev_b = False
    prev_trigger = False
    prev_x = False
    prev_y = False
    recording = False
    ep_dir = None
    csv_writer = None
    csv_file = None
    ep_frame = 0
    record_timer = 0.0

    stop_event = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop_event.set())

    def close_episode():
        nonlocal recording, csv_writer, csv_file, ep_frame, ep_idx
        if csv_writer is not None:
            csv_file.close()
            print(f"[RECORD STOP ] episode {ep_idx} saved ({ep_frame} frames)")
        recording = False
        csv_writer = None
        csv_file = None
        ep_frame = 0
        ep_idx += 1

    try:
        while not stop_event.is_set():
            loop_start = time.time()
            lp, rp, btns = vr.get_state()
            b = bool(btns.get("B", False))
            trigger = bool(btns.get("RTr", False))
            x_press = bool(btns.get("X", False))
            y_press = bool(btns.get("Y", False))

            now = time.time()
            data_fresh = (rp is not None) and (now - vr.last_input_time <= vt.INPUT_TIMEOUT_S)
            if rp is not None:
                vr.last_input_time = now

            # clutch
            if b and not prev_b:
                vr_ref = rp.copy()
                cur_pose = read_tcp(robot)
                tcp_ref = cur_pose.copy()
                governor.reset(cur_pose)
                print("[TELEOP ON]")
            if not b and prev_b:
                print("[TELEOP OFF]")
            prev_b = b

            if gripper is not None and trigger != prev_trigger:
                if trigger:
                    gripper.close()
                else:
                    gripper.open()
            prev_trigger = trigger

            motion_target = None
            cmd_vel = None
            if b and rp is not None and vr_ref is not None:
                dt_gov = (now - prev_loop_t) if prev_loop_t is not None else 1.0 / vt.FPS
                prev_loop_t = now
                if data_fresh:
                    tcp_target = np.empty(7)
                    tcp_target[:3] = (tcp_ref[:3]
                                      + axis_matrix @ (rp[:3, 3] - vr_ref[:3, 3]) * pos_scale)
                    R_vr_ref = vr_ref[:3, :3]
                    R_rp = rp[:3, :3]
                    R_vr_delta = R_rp @ R_vr_ref.T
                    R_robot_delta = axis_matrix @ R_vr_delta @ axis_matrix.T
                    rv = vt.matrix_to_rotvec(R_robot_delta) * rot_scale
                    R_delta = vt.rotvec_to_matrix(rv)
                    R_tcp_ref = vt.quat_to_mat(tcp_ref[3:7])
                    R_target = R_delta @ R_tcp_ref
                    tcp_target[3:7] = vt.mat_to_quat(R_target)
                    motion_target = governor.update(tcp_target, dt_gov)
                    cmd_vel = np.concatenate([governor.v_cmd, governor.w_cmd])
                else:
                    stale_hold = np.empty(7)
                    stale_hold[:3] = governor.p_cmd if governor.p_cmd is not None else cur_pose[:3]
                    stale_hold[3:7] = governor.q_cmd if governor.q_cmd is not None else cur_pose[3:7]
                    motion_target = governor.update(stale_hold, dt_gov)
                    cmd_vel = np.concatenate([governor.v_cmd, governor.w_cmd])
            else:
                motion_target = cur_pose

            if motion_target is not None:
                cur_pose = motion_target.copy()

            if not robot.operational() or not robot.estop_released():
                print("!!! robot not operational; holding !!!")
            else:
                if cmd_vel is not None:
                    vt.send_tcp_target(robot, motion_target, cmd_vel)
                else:
                    vt.send_tcp_target(robot, motion_target)

            if x_press and not prev_x:
                ep_dir = session / f"episode_{ep_idx}"
                ep_dir.mkdir(parents=True, exist_ok=True)
                csv_file = open(ep_dir / "data.csv", "w", newline="")
                csv_writer = csv.writer(csv_file)
                csv_writer.writerow([
                    "timestamp", "frame",
                    "s_x", "s_y", "s_z", "s_qw", "s_qx", "s_qy", "s_qz", "s_gripper",
                    "a_x", "a_y", "a_z", "a_qw", "a_qx", "a_qy", "a_qz", "a_gripper",
                ])
                recording = True
                ep_frame = 0
                record_timer = now
                print("[RECORD START]")
            if y_press and not prev_y:
                if recording:
                    close_episode()
            prev_x, prev_y = x_press, y_press

            if recording and (now - record_timer) >= 1.0 / args.fps:
                record_timer = now
                f_top, f_front = cam.read()
                if f_top is None or f_front is None:
                    continue
                import cv2
                img_dir = ep_dir / "images"
                img_dir.mkdir(exist_ok=True)
                cv2.imwrite(str(img_dir / f"top_{ep_frame:05d}.png"), f_top)
                cv2.imwrite(str(img_dir / f"front_{ep_frame:05d}.png"), f_front)

                real_tcp = read_tcp(robot)
                gpos = read_gripper(gripper)
                state = np.concatenate([real_tcp[:3], real_tcp[3:7], [gpos]])
                act = np.concatenate([motion_target[:3], motion_target[3:7], [gpos]])
                csv_writer.writerow([
                    f"{now:.4f}", ep_frame,
                    *[f"{v:.6f}" for v in state],
                    *[f"{v:.6f}" for v in act],
                ])
                ep_frame += 1

    except KeyboardInterrupt:
        pass
    finally:
        if recording:
            close_episode()
        if gripper is not None:
            try:
                gripper.open()
            except Exception:
                pass
        cam.stop()
        vr.stop()
        robot.Stop()
        print("Done. Raw recording in:", session)


if __name__ == "__main__":
    main()
