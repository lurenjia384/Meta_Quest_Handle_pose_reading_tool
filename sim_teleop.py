import os
import sys
import time
import signal
import argparse
import subprocess
import threading
import xml.etree.ElementTree as ET
from copy import deepcopy
import numpy as np
from scipy.spatial.transform import Rotation as Rot
import mujoco
import mujoco.viewer
SO101_DIR = "./SO101"
XML_PATH = os.path.join(SO101_DIR, "so101_new_calib.xml")
ADB_TAG = 'wE9ryARX'
JOINT_NAMES = [
    "shoulder_pan", "shoulder_lift", "elbow_flex",
    "wrist_flex", "wrist_roll", "gripper",
]
HOME_QPOS = np.array([0.0, -0.35, 1.05, -0.52, 0.0, 0.0])
POS_SCALE = 0.6
VR_TO_ROBOT = np.array([
    [0, 0, -1], 
    [-1, 0, 0], 
    [0, 1, 0], 
])
INVERT_FORWARD_ROLL = False
LEFT_ARM_Y_OFFSET = 0.30
RIGHT_ARM_Y_OFFSET = -0.30 
IK_MAX_ITER = 50
IK_TOL = 1e-3
IK_DAMPING = 0.02
IK_STEP_CLAMP = 0.3
IK_POS_WEIGHT = 1.0
IK_ROT_WEIGHT = 0.8
IK_ROT_CONV_TOL = 0.05
IK_REGULARIZATION = 0.001
IK_MAX_DQ = 0.05
SMOOTH_POS_ALPHA = 0.6
SMOOTH_MAX_ROT_STEP = 0.3
FPS = 30
GRIPPER_OPEN = 1.4
GRIPPER_CLOSED = -0.17

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
        self.left_pose = None
        self.right_pose = None
        self.buttons = {}
        self._running = False
        self._process = None
        self._thread = None

    def start(self):
        result = subprocess.run(['adb', 'devices'], capture_output=True, text=True)
        if 'device' not in result.stdout:
            raise RuntimeError(
                "Quest device not detected. Ensure USB connection + developer mode.")

        subprocess.run([
            'adb', 'shell', 'am', 'start',
            '-n', 'com.rail.oculus.teleop/com.rail.oculus.teleop.MainActivity',
            '-a', 'android.intent.action.MAIN',
            '-c', 'android.intent.category.LAUNCHER',
        ], capture_output=True)

        time.sleep(1)

        self._process = subprocess.Popen(
            ['adb', 'logcat', '-T', '0'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
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
                    if 'l' in transforms:
                        self.left_pose = transforms['l'].copy()
                    if 'r' in transforms:
                        self.right_pose = transforms['r'].copy()
                    self.buttons = btns
            except Exception:
                pass

    def get_state(self):
        with self._lock:
            return (
                self.left_pose.copy() if self.left_pose is not None else None,
                self.right_pose.copy() if self.right_pose is not None else None,
                self.buttons.copy(),
            )

    def stop(self):
        self._running = False
        if self._process:
            self._process.terminate()
        if self._thread:
            self._thread.join(timeout=2)

def _prefix_names(elem, prefix):
    if 'name' in elem.attrib:
        elem.set('name', prefix + elem.get('name'))
    if 'joint' in elem.attrib:
        elem.set('joint', prefix + elem.get('joint'))
    for child in elem:
        _prefix_names(child, prefix)


def create_dual_arm_scene():
    tree = ET.parse(XML_PATH)
    root = tree.getroot()

    compiler = root.find('compiler')
    compiler.set('meshdir', os.path.join(SO101_DIR, 'assets'))
    all_defaults = root.findall('default') 
    worldbody = root.find('worldbody')
    original_body = worldbody.find('body') 
    asset = root.find('asset')
    actuator = root.find('actuator')

    scene = ET.Element('mujoco', {'model': 'dual_arm_so101'})
    scene.append(deepcopy(compiler))
    for d in all_defaults:
        scene.append(deepcopy(d))

    new_worldbody = ET.SubElement(scene, 'worldbody')
    ET.SubElement(new_worldbody, 'light',
                  {'pos': '0 0 3.5', 'dir': '0 0 -1', 'directional': 'true'})

    ET.SubElement(new_worldbody, 'geom',
                  {'name': 'floor', 'size': '0 0 0.05', 'pos': '0 0 0',
                   'type': 'plane', 'rgba': '0.2 0.3 0.4 1'})

    left_body = deepcopy(original_body)
    _prefix_names(left_body, 'left_')
    left_body.set('pos', f'0 {LEFT_ARM_Y_OFFSET} 0')
    new_worldbody.append(left_body)

    right_body = deepcopy(original_body)
    _prefix_names(right_body, 'right_')
    right_body.set('pos', f'0 {RIGHT_ARM_Y_OFFSET} 0')
    new_worldbody.append(right_body)

    scene.append(deepcopy(asset))

    new_actuator = ET.SubElement(scene, 'actuator')
    for prefix in ['left_', 'right_']:
        for act in actuator:
            new_act = deepcopy(act)
            new_act.set('name', prefix + act.get('name'))
            new_act.set('joint', prefix + act.get('joint'))
            new_actuator.append(new_act)
    ET.SubElement(scene, 'equality')
    visual = ET.SubElement(scene, 'visual')
    ET.SubElement(visual, 'global', {'azimuth': '180', 'elevation': '-25'})
    xml_str = ET.tostring(scene, encoding='unicode')
    return xml_str


def dls_ik(model, data, site_id, dof_start, n_dof,
           target_pos, target_rot, q_init,
           max_iter=IK_MAX_ITER, tol=IK_TOL, damping=IK_DAMPING,
           pos_weight=IK_POS_WEIGHT, rot_weight=IK_ROT_WEIGHT,
           regularization=IK_REGULARIZATION):
    q = q_init.copy()
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    W = np.diag([pos_weight] * 3 + [rot_weight] * 3)
    orient_mask = np.array([1.0, 1.0, 0.0])
    for _ in range(max_iter):
        data.qpos[dof_start:dof_start + n_dof] = q
        mujoco.mj_kinematics(model, data)
        mujoco.mj_comPos(model, data)
        current_pos = data.site_xpos[site_id].copy()
        current_rot = data.site_xmat[site_id].reshape(3, 3).copy()
        pos_err = target_pos - current_pos
        rot_err_full = (Rot.from_matrix(target_rot)
                        * Rot.from_matrix(current_rot).inv()).as_rotvec()
        rot_err = current_rot @ (current_rot.T @ rot_err_full * orient_mask)
        err = np.concatenate([pos_err, rot_err])
        if (np.linalg.norm(pos_err) < tol
                and np.linalg.norm(rot_err) < IK_ROT_CONV_TOL):
            break
        mujoco.mj_jacSite(model, data, jacp, jacr, site_id)
        J_full = np.vstack([jacp, jacr])
        J = J_full[:, dof_start:dof_start + n_dof]
        JTW = J.T @ W
        JTJ = JTW @ J
        rhs = JTW @ err - regularization * (q - q_init)
        dq = np.linalg.solve(JTJ + (damping ** 2) * np.eye(n_dof)
                             + regularization * np.eye(n_dof), rhs)
        step_norm = np.linalg.norm(dq)
        if step_norm > IK_STEP_CLAMP:
            dq = dq * (IK_STEP_CLAMP / step_norm)
        q = q + dq
        for j in range(n_dof):
            jid = dof_start + j
            lo, hi = model.jnt_range[jid]
            if lo < hi:
                q[j] = np.clip(q[j], lo, hi)
    return q


def rate_limit_joints(q_new, q_prev, max_dq=IK_MAX_DQ):
    dq = q_new - q_prev
    norm = np.linalg.norm(dq)
    if norm > max_dq:
        dq = dq * (max_dq / norm)
    return q_prev + dq


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


def slerp_rot(R1, R2, alpha):
    r1 = Rot.from_matrix(R1)
    r2 = Rot.from_matrix(R2)
    return (r1 * (r1.inv() * r2) ** alpha).as_matrix()


def smooth_target(prev_pos, prev_rot, new_pos, new_rot,
                  pos_alpha=0.6, max_rot_step=0.3):
    if prev_pos is None or prev_rot is None:
        return new_pos.copy(), new_rot.copy()
    smooth_pos = prev_pos * (1 - pos_alpha) + new_pos * pos_alpha
    rel = Rot.from_matrix(new_rot @ prev_rot.T)
    angle = np.linalg.norm(rel.as_rotvec())
    if angle > 1e-6 and angle > max_rot_step:
        alpha = max_rot_step / angle
        smooth_rot = slerp_rot(prev_rot, new_rot, alpha)
    else:
        smooth_rot = new_rot.copy()
    return smooth_pos, smooth_rot

def get_arm_joint_info(model, prefix):
    qpos_addrs = []
    dof_addrs = []
    joint_ranges = []
    for jn in JOINT_NAMES:
        full_name = f"{prefix}{jn}"
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, full_name)
        if jid < 0:
            raise ValueError(f"Joint '{full_name}' not found in model")
        qpos_addrs.append(model.jnt_qposadr[jid])
        dof_addrs.append(model.jnt_dofadr[jid])
        joint_ranges.append(model.jnt_range[jid].copy())
    return np.array(qpos_addrs), np.array(dof_addrs), np.array(joint_ranges)

def main():
    default_scale = globals().get('POS_SCALE', 0.6)
    default_smooth = globals().get('SMOOTH_POS_ALPHA', 0.6)
    default_rot_step = globals().get('SMOOTH_MAX_ROT_STEP', 0.3)
    parser = argparse.ArgumentParser(
        description="VR Dual-Arm Teleoperation (MuJoCo Simulation)")
    parser.add_argument('--scale', type=float, default=default_scale,
                        help=f'End-effector motion scale (default {default_scale})')
    parser.add_argument('--smooth', type=float, default=default_smooth,
                        help=f'Smoothing filter ratio 0..1 (default {default_smooth}; '
                             f'higher = more responsive, lower = smoother)')
    parser.add_argument('--rot-step', type=float, default=default_rot_step,
                        help=f'Max target rotation change per frame, rad '
                             f'(default {default_rot_step})')
    args = parser.parse_args()

    global POS_SCALE, SMOOTH_POS_ALPHA, SMOOTH_MAX_ROT_STEP
    POS_SCALE = args.scale
    SMOOTH_POS_ALPHA = max(0.0, min(1.0, args.smooth))
    SMOOTH_MAX_ROT_STEP = args.rot_step

    print("=" * 60)
    print("  VR Dual-Arm Teleoperation (MuJoCo Simulation)")
    print("=" * 60)
    print(f"  Scale:  {POS_SCALE}   Smooth: {SMOOTH_POS_ALPHA}   "
          f"RotStep: {SMOOTH_MAX_ROT_STEP} rad")
    print("\n[1/4] Building dual-arm MuJoCo scene...")
    xml_str = create_dual_arm_scene()
    model = mujoco.MjModel.from_xml_string(xml_str)
    data = mujoco.MjData(model)
    print(f"  Model loaded: {model.nq} DOF, {model.nu} actuators, {model.nbody} bodies")
    left_qpos, left_dof, left_range = get_arm_joint_info(model, 'left_')
    right_qpos, right_dof, right_range = get_arm_joint_info(model, 'right_')
    left_site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'left_gripperframe')
    right_site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'right_gripperframe')
    if left_site < 0 or right_site < 0:
        raise RuntimeError("gripperframe sites not found")
    for i, addr in enumerate(left_qpos):
        data.qpos[addr] = HOME_QPOS[i]
    for i, addr in enumerate(right_qpos):
        data.qpos[addr] = HOME_QPOS[i]
    mujoco.mj_kinematics(model, data)
    mujoco.mj_comPos(model, data)
    left_ref_pos = data.site_xpos[left_site].copy()
    left_ref_rot = data.site_xmat[left_site].reshape(3, 3).copy()
    right_ref_pos = data.site_xpos[right_site].copy()
    right_ref_rot = data.site_xmat[right_site].reshape(3, 3).copy()
    print(f"  Left  arm home EE: pos={left_ref_pos}, dof_start={left_dof[0]}")
    print(f"  Right arm home EE: pos={right_ref_pos}, dof_start={right_dof[0]}")
    print("\n[2/4] Connecting to Quest VR...")
    vr = VRPoseReader()
    vr.start()
    print("  VR reader started. Waiting for pose data...")
    while True:
        lp, rp, _ = vr.get_state()
        if lp is not None or rp is not None:
            break
        time.sleep(0.5)
    print("  VR data received!")
    print("\n[3/4] Launching MuJoCo viewer...")
    print("\n  Controls:")
    print("    RIGHT GRIP (RG) HOLD : teleop active (release = pause)")
    print("    LEFT  GRIP (LG) press : reset arms to home")
    print("    RIGHT TRIG (RTr)      : right gripper close")
    print("    LEFT  TRIG (LTr)      : left  gripper close")
    print("    Ctrl+C                : quit")
    print()
    left_q = HOME_QPOS.copy()
    right_q = HOME_QPOS.copy()
    left_ref_pos = data.site_xpos[left_site].copy()
    left_ref_rot = data.site_xmat[left_site].reshape(3, 3).copy()
    right_ref_pos = data.site_xpos[right_site].copy()
    right_ref_rot = data.site_xmat[right_site].reshape(3, 3).copy()
    vr_left_ref = None
    vr_right_ref = None
    left_target_pos = None
    left_target_rot = None
    right_target_pos = None
    right_target_rot = None
    prev_rg = False
    prev_lg = False
    frame_count = 0
    fps_timer = time.time()
    with mujoco.viewer.launch_passive(model, data) as viewer:
        try:
            while viewer.is_running():
                loop_start = time.time()
                lp, rp, btns = vr.get_state()
                rg = btns.get('RG', False)
                lg = btns.get('LG', False)
                if lg and not prev_lg:
                    left_q = HOME_QPOS.copy()
                    right_q = HOME_QPOS.copy()
                    for i, addr in enumerate(left_qpos):
                        data.qpos[addr] = left_q[i]
                    for i, addr in enumerate(right_qpos):
                        data.qpos[addr] = right_q[i]
                    mujoco.mj_kinematics(model, data)
                    mujoco.mj_comPos(model, data)
                    if lp is not None:
                        vr_left_ref = lp.copy()
                    if rp is not None:
                        vr_right_ref = rp.copy()
                    left_ref_pos = data.site_xpos[left_site].copy()
                    left_ref_rot = data.site_xmat[left_site].reshape(3, 3).copy()
                    right_ref_pos = data.site_xpos[right_site].copy()
                    right_ref_rot = data.site_xmat[right_site].reshape(3, 3).copy()
                    left_target_pos = left_ref_pos.copy()
                    left_target_rot = left_ref_rot.copy()
                    right_target_pos = right_ref_pos.copy()
                    right_target_rot = right_ref_rot.copy()
                    print("  [RESET] arms back to home, refs re-anchored")
                prev_lg = lg
                if rg and not prev_rg:
                    if lp is not None:
                        vr_left_ref = lp.copy()
                    if rp is not None:
                        vr_right_ref = rp.copy()
                    print("  [TELEOP ON] holding right grip")
                prev_rg = rg
                if rg and lp is not None and vr_left_ref is not None:
                    tgt_pos, tgt_rot = map_vr_to_robot(
                        lp, vr_left_ref, left_ref_pos, left_ref_rot)
                    tgt_pos, tgt_rot = smooth_target(
                        left_target_pos, left_target_rot, tgt_pos, tgt_rot,
                        pos_alpha=SMOOTH_POS_ALPHA, max_rot_step=SMOOTH_MAX_ROT_STEP)
                    left_target_pos, left_target_rot = tgt_pos.copy(), tgt_rot.copy()
                    left_ik = dls_ik(
                        model, data, left_site, left_dof[0], 6,
                        tgt_pos, tgt_rot, left_q)
                    left_ik = rate_limit_joints(left_ik, left_q)
                    left_q[:] = left_ik
                    left_q[5] = GRIPPER_CLOSED if btns.get('LTr', False) else GRIPPER_OPEN
                    for i, addr in enumerate(left_qpos):
                        data.qpos[addr] = left_q[i]

                if rg and rp is not None and vr_right_ref is not None:
                    tgt_pos, tgt_rot = map_vr_to_robot(
                        rp, vr_right_ref, right_ref_pos, right_ref_rot)
                    tgt_pos, tgt_rot = smooth_target(
                        right_target_pos, right_target_rot, tgt_pos, tgt_rot,
                        pos_alpha=SMOOTH_POS_ALPHA, max_rot_step=SMOOTH_MAX_ROT_STEP)
                    right_target_pos, right_target_rot = tgt_pos.copy(), tgt_rot.copy()
                    right_ik = dls_ik(
                        model, data, right_site, right_dof[0], 6,
                        tgt_pos, tgt_rot, right_q)
                    right_ik = rate_limit_joints(right_ik, right_q)
                    right_q[:] = right_ik
                    right_q[5] = GRIPPER_CLOSED if btns.get('RTr', False) else GRIPPER_OPEN
                    for i, addr in enumerate(right_qpos):
                        data.qpos[addr] = right_q[i]
                mujoco.mj_kinematics(model, data)
                viewer.sync()
                frame_count += 1
                now = time.time()
                if now - fps_timer > 5.0:
                    fps_actual = frame_count / (now - fps_timer)
                    frame_count = 0
                    fps_timer = now
                    status = "ACTIVE" if rg else "IDLE"
                    print(f"  [{status}] FPS: {fps_actual:.1f}")
                elapsed = time.time() - loop_start
                time.sleep(max(0, 1.0 / FPS - elapsed))
        except KeyboardInterrupt:
            print("\n\nStopping...")
        finally:
            vr.stop()
            print("VR reader stopped. Done.")


if __name__ == '__main__':
    main()
