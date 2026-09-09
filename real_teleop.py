import os
import sys
import time
import signal
import subprocess
import threading
import argparse
import types

import numpy as np
from scipy.spatial.transform import Rotation as Rot
import placo
if 'IPython' not in sys.modules:
    _stub = types.ModuleType('IPython')
    _stub.embed = lambda *a, **k: None
    _stub.get_ipython = lambda: None
    sys.modules['IPython'] = _stub
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.robots.so_follower.config_so_follower import SO101FollowerConfig
from lerobot.robots.so_follower.so_follower import SO101Follower
SO101_DIR = "/home/robot/ycc/SO101"
URDF_PATH = os.path.join(SO101_DIR, "so101_new_calib.urdf")
ADB_TAG = 'wE9ryARX'
MOTOR_NAMES = [
    "shoulder_pan", "shoulder_lift", "elbow_flex",
    "wrist_flex", "wrist_roll", "gripper",
]
ROBOT_ID = "my_awesome_follower_arm"
FPS = 30
HOME_QPOS_RAD = np.array([0.0, -0.35, 1.05, -0.52, 0.0, 0.0])
GRIPPER_OPEN_DEG = 80.0
GRIPPER_CLOSED_DEG = 0.0
POS_SCALE = 0.6
VR_TO_ROBOT = np.array([
    [0, 0, -1],  
    [-1, 0, 0], 
    [0, 1, 0], 
])
INVERT_FORWARD_ROLL = False
IK_ROT_WEIGHT = 0.8
IK_REGULARIZATION = 0.001
IK_MAX_ITER = 10
MAX_DQ_DEG = 3.0 

def parse_transforms(data_string):
    try:
        transforms_string, buttons_string = data_string.split('&')
    except ValueError:
        return None, None

    transforms = {}
    for pair_string in transforms_string.split('|'):
        transform = np.empty((4, 4))
        pair = pair_string.split(':')
        if len(pair) != 2:
            continue
        left_right_char = pair[0]
        values = pair[1].split(' ')

        c, r, count = 0, 0, 0
        for value in values:
            if not value:
                continue
            transform[r][c] = float(value)
            c += 1
            if c >= 4:
                c = 0
                r += 1
            count += 1

        if count == 16:
            transforms[left_right_char] = transform

    return transforms, buttons_string


def parse_buttons(text):
    split_text = text.split(',')
    buttons = {}
    if 'R' in split_text:
        split_text.remove('R')
        buttons.update({'A': False, 'B': False, 'RThU': False,
                         'RJ': False, 'RG': False, 'RTr': False})
    if 'L' in split_text:
        split_text.remove('L')
        buttons.update({'X': False, 'Y': False, 'LThU': False,
                         'LJ': False, 'LG': False, 'LTr': False})
    for key in list(buttons.keys()):
        if key in split_text:
            buttons[key] = True
            split_text.remove(key)
    for elem in split_text:
        split_elem = elem.split(' ')
        if len(split_elem) < 2:
            continue
        key = split_elem[0]
        value = tuple(float(x) for x in split_elem[1:])
        buttons[key] = value
    return buttons

class VRPoseReader:
    def __init__(self):
        self._lock = threading.Lock()
        self.right_pose = None
        self.buttons = {}
        self._running = False
        self._process = None
        self._thread = None

    def start(self):
        result = subprocess.run(['adb', 'devices'], capture_output=True, text=True)
        if 'device' not in result.stdout:
            raise RuntimeError("Quest device not detected.")

        subprocess.run([
            'adb', 'shell', 'am', 'start',
            '-n', 'com.rail.oculus.teleop/com.rail.oculus.teleop.MainActivity',
            '-a', 'android.intent.action.MAIN',
            '-c', 'android.intent.category.LAUNCHER',
        ], capture_output=True)
        time.sleep(1)

        self._process = subprocess.Popen(
            ['adb', 'logcat', '-T', '0'],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True, bufsize=1,
        )
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _read_loop(self):
        for line in self._process.stdout:
            if not self._running:
                break
            line = line.strip()
            if ADB_TAG not in line:
                continue
            try:
                data = line.split(ADB_TAG + ': ')[1]
                transforms, buttons_string = parse_transforms(data)
                if transforms is None:
                    continue
                btns = parse_buttons(buttons_string) if buttons_string else {}
                with self._lock:
                    if 'r' in transforms:
                        self.right_pose = transforms['r'].copy()
                    self.buttons = btns
            except Exception:
                pass

    def get_state(self):
        with self._lock:
            return (
                self.right_pose.copy() if self.right_pose is not None else None,
                self.buttons.copy(),
            )

    def stop(self):
        self._running = False
        if self._process:
            self._process.terminate()
        if self._thread:
            self._thread.join(timeout=2)

class SO101IKSolver:
    def __init__(self, urdf_path, home_qpos):
        self.robot = placo.RobotWrapper(urdf_path)
        self.joint_names = list(self.robot.joint_names())
        self.solver = placo.KinematicsSolver(self.robot)
        self.solver.mask_fbase(True)
        self._home = home_qpos.copy()
        for i, jn in enumerate(self.joint_names):
            self.robot.set_joint(jn, float(home_qpos[i]))
        self.robot.update_kinematics()
        T_home = self.robot.get_T_world_frame('gripper_frame_link')
        self.home_pos = T_home[:3, 3].copy()
        self.home_rot = T_home[:3, :3].copy()
        self.pos_task = self.solver.add_position_task(
            'gripper_frame_link', self.home_pos.copy())
        self.pos_task.configure('pos', 'soft', 1.0)
        self.rot_task = self.solver.add_orientation_task(
            'gripper_frame_link', self.home_rot.copy())
        self.rot_task.configure('rot', 'soft', IK_ROT_WEIGHT)
        self.rot_task.mask.set_axises('xy')
        self.reg_task = self.solver.add_regularization_task(IK_REGULARIZATION)

    def solve(self, target_pos, target_rot, max_iter=IK_MAX_ITER):
        self.pos_task.target_world = target_pos
        self.rot_task.R_world_frame = target_rot

        for _ in range(max_iter):
            self.solver.solve(True)
            self.robot.update_kinematics()
        q = np.array([self.robot.get_joint(jn) for jn in self.joint_names])
        return q

def _invert_forward_roll(rot_matrix):
    from scipy.spatial.transform import Rotation as Rot
    e = Rot.from_matrix(rot_matrix).as_euler('xyz')
    e[0] = -e[0]
    return Rot.from_euler('xyz', e).as_matrix()

def map_vr_to_robot(vr_pose, vr_ref, arm_ref_pos, arm_ref_rot,
                    scale=POS_SCALE, vr_to_robot=VR_TO_ROBOT,
                    invert_forward_roll=INVERT_FORWARD_ROLL):
    T_rel = np.linalg.inv(vr_ref) @ vr_pose

    delta_pos_vr = T_rel[:3, 3]
    delta_rot_vr = T_rel[:3, :3] 
    delta_pos_robot = vr_to_robot @ delta_pos_vr * scale
    delta_rot_robot = vr_to_robot @ delta_rot_vr @ vr_to_robot.T
    if invert_forward_roll:
        delta_rot_robot = _invert_forward_roll(delta_rot_robot)
    target_pos = arm_ref_pos + delta_pos_robot
    target_rot = delta_rot_robot @ arm_ref_rot
    return target_pos, target_rot

def connect_robot(port, execute=True):
    cfg = SO101FollowerConfig(
        port=port,
        id=ROBOT_ID,
        use_degrees=True,
        cameras={},
    )
    robot = SO101Follower(cfg)
    robot.connect(calibrate=False)
    print(f"Robot connected: port={port}, execute={execute}")
    return robot

def main():
    parser = argparse.ArgumentParser(
        description="VR Teleoperation — Real SO-101 Robot")
    parser.add_argument('--port', default='/dev/ttyACM0',
                        help='Motor serial port (default /dev/ttyACM0)')
    parser.add_argument('--execute', action='store_true',
                        help='Send actions to real robot (default: dry-run)')
    parser.add_argument('--scale', type=float, default=POS_SCALE,
                        help=f'End-effector motion scale (default {POS_SCALE})')
    parser.add_argument('--smooth', type=float, default=1.0,
                        help='Smoothing ratio 0..1 applied to the IK joint '
                             'output per frame (default 1.0 = no smoothing; '
                             'lower = smoother/slower response)')
    parser.add_argument('--gripper-test', action='store_true',
                        help='Gripper self-test: sweep open/close and print, '
                             'no VR needed')
    args = parser.parse_args()
    smooth_ratio = max(0.0, min(1.0, args.smooth))
    max_dq = MAX_DQ_DEG * smooth_ratio
    print("=" * 60)
    print("  VR Teleoperation — Real SO-101 Robot")
    print("=" * 60)
    print(f"  Port:    {args.port}")
    print(f"  Execute: {args.execute}")
    print(f"  Scale:   {args.scale}")
    print(f"  Smooth:  {smooth_ratio} (max step {max_dq:.1f}°/frame)")
    if args.gripper_test:
        robot = connect_robot(args.port, args.execute)
        print("\n  Gripper self-test (Ctrl+C to stop):")
        print(f"  GRIPPER_OPEN_DEG   = {GRIPPER_OPEN_DEG}° (released)")
        print(f"  GRIPPER_CLOSED_DEG = {GRIPPER_CLOSED_DEG}° (pressed)")
        print("  Observe which position the jaws are in for each value.")
        print("  If OPEN looks closed (or vice versa), swap the two config")
        print("  values at the top of this script.\n")
        try:
            while True:
                action = {f"{m}.pos": float(np.rad2deg(HOME_QPOS_RAD)[i])
                          for i, m in enumerate(MOTOR_NAMES)}
                action['gripper.pos'] = GRIPPER_OPEN_DEG
                if args.execute:
                    robot.send_action(action)
                print(f"  gripper = {GRIPPER_OPEN_DEG:5.1f}°  (OPEN)")
                time.sleep(2)
                action['gripper.pos'] = GRIPPER_CLOSED_DEG
                if args.execute:
                    robot.send_action(action)
                print(f"  gripper = {GRIPPER_CLOSED_DEG:5.1f}°  (CLOSED)")
                time.sleep(2)
        except KeyboardInterrupt:
            print("\n  Gripper test stopped.")
        robot.disconnect()
        return
    print("\n[1/4] Initializing placo IK solver...")
    ik = SO101IKSolver(URDF_PATH, HOME_QPOS_RAD)
    print(f"  Home EE pos: {ik.home_pos}")
    print(f"  Joints: {ik.joint_names}")
    print("\n[2/4] Connecting to SO-101 robot...")
    robot = connect_robot(args.port, args.execute)
    home_deg = np.rad2deg(HOME_QPOS_RAD)
    home_action = {f"{m}.pos": float(home_deg[i]) for i, m in enumerate(MOTOR_NAMES)}
    if args.execute:
        robot.send_action(home_action)
        time.sleep(2)
    print(f"  Home position sent (deg): {home_deg.round(1)}")
    print("\n[3/4] Connecting to Quest VR...")
    vr = VRPoseReader()
    vr.start()
    print("  Waiting for VR pose data...")
    while True:
        rp, _ = vr.get_state()
        if rp is not None:
            break
        time.sleep(0.5)
    print("  VR data received!")
    print("\n[4/4] Ready for teleoperation.")
    print("\n  Controls:")
    print("    RIGHT GRIP (RG) HOLD : teleop active (release = pause)")
    print("    LEFT  GRIP (LG) press : reset robot to home")
    print("    RIGHT TRIG (RTr)      : close gripper")
    print("    Ctrl+C                : quit")
    print()
    ik.robot.update_kinematics()
    T_ref = ik.robot.get_T_world_frame('gripper_frame_link')
    arm_ref_pos = T_ref[:3, 3].copy()
    arm_ref_rot = T_ref[:3, :3].copy()

    vr_ref = None
    prev_rg = False
    prev_lg = False
    prev_q_deg = np.rad2deg(HOME_QPOS_RAD)

    frame_count = 0
    fps_timer = time.time()

    try:
        while True:
            loop_start = time.time()

            rp, btns = vr.get_state()
            rg = btns.get('RG', False)
            lg = btns.get('LG', False)

            if lg and not prev_lg:
                if args.execute:
                    obs = robot.get_observation()
                    start_q = np.array(
                        [obs[f"{m}.pos"] for m in MOTOR_NAMES], dtype=np.float32)
                    target_q = np.rad2deg(HOME_QPOS_RAD)
                    n_steps = int(FPS)
                    for s in range(1, n_steps + 1):
                        interp = start_q + (target_q - start_q) * (s / n_steps)
                        action = {f"{m}.pos": float(interp[i])
                                  for i, m in enumerate(MOTOR_NAMES)}
                        robot.send_action(action)
                        time.sleep(1.0 / FPS)
                print("  [RESET] robot back to home")
                for i, jn in enumerate(ik.joint_names):
                    ik.robot.set_joint(jn, float(HOME_QPOS_RAD[i]))
                ik.robot.update_kinematics()
                T_ref = ik.robot.get_T_world_frame('gripper_frame_link')
                arm_ref_pos = T_ref[:3, 3].copy()
                arm_ref_rot = T_ref[:3, :3].copy()
                prev_q_deg = np.rad2deg(HOME_QPOS_RAD)
                if rp is not None:
                    vr_ref = rp.copy()
            prev_lg = lg
            
            if rg and not prev_rg:
                if rp is not None:
                    vr_ref = rp.copy()
                if args.execute:
                    obs = robot.get_observation()
                    real_q_deg = np.array(
                        [obs[f"{m}.pos"] for m in MOTOR_NAMES], dtype=np.float64)
                    for i, jn in enumerate(ik.joint_names):
                        ik.robot.set_joint(jn, float(np.deg2rad(real_q_deg[i])))
                else:
                    for i, jn in enumerate(ik.joint_names):
                        ik.robot.set_joint(jn, float(HOME_QPOS_RAD[i]))
                ik.robot.update_kinematics()
                T_ref = ik.robot.get_T_world_frame('gripper_frame_link')
                arm_ref_pos = T_ref[:3, 3].copy()
                arm_ref_rot = T_ref[:3, :3].copy()
                if args.execute:
                    obs = robot.get_observation()
                    prev_q_deg = np.array(
                        [obs[f"{m}.pos"] for m in MOTOR_NAMES], dtype=np.float64)
                else:
                    prev_q_deg = np.rad2deg(HOME_QPOS_RAD)
                print("  [TELEOP ON] holding right grip")
            prev_rg = rg
            if rg and rp is not None and vr_ref is not None:
                target_pos, target_rot = map_vr_to_robot(
                    rp, vr_ref, arm_ref_pos, arm_ref_rot, scale=args.scale)
                q_rad = ik.solve(target_pos, target_rot, max_iter=IK_MAX_ITER)
                q_deg = np.rad2deg(q_rad)
                if btns.get('RTr', False):
                    q_deg[5] = GRIPPER_CLOSED_DEG
                else:
                    q_deg[5] = GRIPPER_OPEN_DEG
                dq = q_deg - prev_q_deg
                dq_norm = np.linalg.norm(dq)
                if dq_norm > max_dq:
                    q_deg = prev_q_deg + dq * (max_dq / dq_norm)
                prev_q_deg = q_deg.copy()
                action = {f"{m}.pos": float(q_deg[i])
                          for i, m in enumerate(MOTOR_NAMES)}

                if args.execute:
                    robot.send_action(action)
                else:
                    if frame_count % 30 == 0:
                        print(f"  q_deg: {q_deg.round(1)}  "
                              f"target_pos: {target_pos.round(4)}")
            frame_count += 1
            now = time.time()
            if now - fps_timer > 5.0:
                fps = frame_count / (now - fps_timer)
                frame_count = 0
                fps_timer = now
                status = "ACTIVE" if rg else "IDLE"
                print(f"  [{status}] FPS: {fps:.1f}")
            elapsed = time.time() - loop_start
            time.sleep(max(0, 1.0 / FPS - elapsed))

    except KeyboardInterrupt:
        print("\n\nStopping...")
    finally:
        if args.execute:
            robot.send_action(home_action)
            time.sleep(1)
        robot.disconnect()
        vr.stop()
        print("Robot disconnected. VR stopped. Done.")

if __name__ == '__main__':
    main()
