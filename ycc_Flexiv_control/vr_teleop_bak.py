import argparse
import collections
import json
import math
import signal
import subprocess
import sys
import threading
import time

import numpy as np
import flexivrdk

TAG = "wE9ryARX"
APP_COMPONENT = "com.rail.oculus.teleop/com.rail.oculus.teleop.MainActivity"
FPS = 1000.0

VR_TO_ROBOT = np.array([
    [0.0, 0.0, -1.0],
    [-1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
])


def load_axis_matrix(path_text):
    if not path_text:
        return VR_TO_ROBOT.copy()
    with open(path_text, "r", encoding="utf-8") as handle:
        payload = json.load(handle)
    matrix = np.asarray(payload["axis_matrix"], dtype=np.float64).reshape(3, 3)
    if not np.isfinite(matrix).all() or not np.allclose(matrix.T @ matrix, np.eye(3), atol=0.05):
        raise ValueError("axis matrix must be orthonormal 3x3: %s" % path_text)
    print("  axis matrix loaded from {} (operator={})".format(
        path_text, payload.get("operator", "<none>")))
    return matrix

POS_SCALE = 0.30  
ROT_SCALE = 0.30 
MAX_STEP_M = 0.0005 
MAX_SPEED_MPS = 0.05  
MAX_ROT_STEP_RAD = 0.004 
MAX_ROT_SPEED_RAD_S = 0.50 
MAX_REL_TRANSLATION_M = 0.15
MAX_REL_ROTATION_RAD = math.radians(20.0)
INPUT_TIMEOUT_S = 0.15
HANDLE_SLOW_VEL = 0.15
MIN_POS_SCALE = 0.05
HANDLE_SMOOTH = 0.5
HANDLE_MAX_DISP = 0.004
WS_MIN = np.array([0.20, -0.50, 0.05])
WS_MAX = np.array([0.80, 0.30, 0.50])

# Reference governor limits (per the optimization doc 4.2):
# keeps commanded velocity AND acceleration continuous so the arm never has
# to sprint after a raw VR target (which caused the "speed can't keep up"
# feeling). Start conservative, raise gradually.
GOV_V_MAX = 0.12        # m/s   (doc start range 0.10-0.15)
GOV_A_MAX = 0.5         # m/s^2 (doc start range 0.4-0.6)
GOV_W_MAX = 0.6         # rad/s (doc start range 0.5-0.8)
GOV_ALPHA_MAX = 1.5     # rad/s^2 (doc start range 1.5-2.0)

def parse_transforms(data_string):
    try:
        transforms_string, buttons_string = data_string.split("&")
    except ValueError:
        return None, None
    transforms = {}
    for pair_string in transforms_string.split("|"):
        transform = np.empty((4, 4))
        pair = pair_string.split(":")
        if len(pair) != 2:
            continue
        left_right_char = pair[0]
        values = pair[1].split(" ")
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
    parts = [p.strip() for p in text.split(",") if p.strip()]
    result = {"A": False, "B": False, "RTr": False, "RG": False}
    for part in parts:
        fields = part.split()
        if not fields:
            continue
        key = fields[0]
        if key in ("R", "L"):
            continue
        if len(fields) == 1:
            result[key] = True
        else:
            try:
                result[key] = tuple(float(v) for v in fields[1:])
            except ValueError:
                continue
    return result


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
        result = subprocess.run(["adb", "devices"], capture_output=True, text=True)
        if "device" not in result.stdout:
            raise RuntimeError("Quest device not detected (adb devices).")
        subprocess.run([
            "adb", "shell", "am", "start",
            "-n", APP_COMPONENT,
            "-a", "android.intent.action.MAIN",
            "-c", "android.intent.category.LAUNCHER",
        ], capture_output=True)
        time.sleep(1)
        self._process = subprocess.Popen(
            ["adb", "logcat", "-T", "0"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        )
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _read_loop(self):
        for raw in self._process.stdout:
            if not self._running:
                break
            # logcat may contain non-UTF-8 bytes; decode lossily
            try:
                line = raw.decode("utf-8", errors="replace").strip()
            except Exception:
                continue
            if TAG not in line:
                continue
            try:
                data = line.split(TAG + ": ")[1]
                transforms, buttons_string = parse_transforms(data)
                if transforms is None:
                    continue
                btns = parse_buttons(buttons_string) if buttons_string else {}
                with self._lock:
                    if "l" in transforms:
                        self.left_pose = transforms["l"].copy()
                    if "r" in transforms:
                        self.right_pose = transforms["r"].copy()
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

def wait_until_operational(robot, timeout_s=15.0):
    deadline = time.time() + timeout_s
    while not robot.operational():
        if time.time() > deadline:
            raise TimeoutError("robot not operational after {}s".format(timeout_s))
        time.sleep(0.2)


def read_tcp(robot):
    s = robot.states()
    return np.asarray(s.tcp_pose, dtype=np.float64).reshape(7)


class ReferenceGovernor:
    """
    Second-order velocity/acceleration limiter (doc 4.2).

    Maintains p_cmd / v_cmd so the commanded pose is velocity- and
    acceleration-continuous even when the raw VR target jumps around. The arm
    then always has a smooth, followable trajectory instead of being asked to
    sprint after every raw target (which caused "speed can't keep up").

    Discrete update (translation; rotation via angular-vel limit + integrate):
        error  = p_raw - p_cmd
        v_ref  = error / tau
        v_ref  = limit_norm(v_ref, v_max)
        dv     = v_ref - v_cmd
        dv     = limit_norm(dv, a_max * dt)
        v_cmd += dv
        p_cmd += v_cmd * dt
    """
    def __init__(self, tau=0.10,
                 v_max=GOV_V_MAX, a_max=GOV_A_MAX,
                 w_max=GOV_W_MAX, alpha_max=GOV_ALPHA_MAX,
                 dt=1.0 / 60.0):
        self.tau = tau
        self.v_max = v_max
        self.a_max = a_max
        self.w_max = w_max
        self.alpha_max = alpha_max
        # FIXED internal integration step. The governor must NOT use the
        # caller's dt: at FPS=1000 the caller dt=0.001 makes the acceleration
        # integration 16x too slow AND the feed-forward velocity (derived from
        # the output step) explodes. A fixed 1/60s step keeps the dynamics
        # identical no matter how often update() is called.
        self.dt = dt
        self.p_cmd = None
        self.v_cmd = np.zeros(3)
        self.q_cmd = None
        self.w_cmd = np.zeros(3)

    def _limit(self, vec, max_norm):
        n = np.linalg.norm(vec)
        if n > max_norm and n > 1e-9:
            return vec * (max_norm / n)
        return vec

    def reset(self, pose):
        self.p_cmd = pose[:3].copy()
        self.v_cmd = np.zeros(3)
        self.q_cmd = pose[3:7].copy()
        self.w_cmd = np.zeros(3)

    def update(self, target_pose, dt=None):
        """Step the governor by one internal integration step."""
        if self.p_cmd is None:
            self.reset(target_pose)
            return target_pose.copy()
        dt = self.dt  # fixed internal step, ignore caller dt

        # translation
        error = target_pose[:3] - self.p_cmd
        v_ref = self._limit(error / self.tau, self.v_max)
        dv = self._limit(v_ref - self.v_cmd, self.a_max * dt)
        self.v_cmd += dv
        self.p_cmd += self.v_cmd * dt

        # rotation: angular-velocity reference, accel-limit, integrate quat
        q_t = target_pose[3:7] / np.linalg.norm(target_pose[3:7])
        q_c = self.q_cmd / np.linalg.norm(self.q_cmd)
        q_inv = np.array([q_c[0], -q_c[1], -q_c[2], -q_c[3]])
        q_rel = quat_multiply(q_t, q_inv)
        theta = 2.0 * math.acos(np.clip(q_rel[0], -1.0, 1.0))
        axis = np.zeros(3)
        if theta > 1e-6:
            axis = q_rel[1:4] / np.linalg.norm(q_rel[1:4])
        w_ref = self._limit(axis * (theta / self.tau), self.w_max)
        dw = self._limit(w_ref - self.w_cmd, self.alpha_max * dt)
        self.w_cmd += dw
        dq = self.w_cmd * dt
        angle = np.linalg.norm(dq)
        if angle > 1e-9:
            ax = dq / angle
            q_step = np.array([math.cos(angle / 2.0),
                               ax[0] * math.sin(angle / 2.0),
                               ax[1] * math.sin(angle / 2.0),
                               ax[2] * math.sin(angle / 2.0)])
            self.q_cmd = quat_multiply(q_step, self.q_cmd)
        self.q_cmd /= np.linalg.norm(self.q_cmd)

        pose = np.empty(7)
        pose[:3] = self.p_cmd
        pose[3:7] = self.q_cmd
        return pose


def quat_multiply(q1, q2):
    """Hamilton product of quaternions (qwxyz)."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def send_tcp_target(robot, pose, prev_pose=None, dt=None,
                     max_lin_vel=0.15, max_ang_vel=5.0):
    """Send a Cartesian target with an explicit feed-forward velocity.

    Velocity is derived from (pose - prev_pose)/dt so the arm moves at the
    commanded speed instead of relying on its internal trajectory
    interpolation (which felt sluggish).
    """
    if prev_pose is not None and dt is not None and dt > 1e-6:
        vel = np.zeros(6)
        vel[:3] = (pose[:3] - prev_pose[:3]) / dt
        Rc = quat_to_mat(prev_pose[3:7])
        Rt = quat_to_mat(pose[3:7])
        rv = matrix_to_rotvec(Rt @ Rc.T)
        vel[3:] = rv / dt
        vel_lin = np.linalg.norm(vel[:3])
        if vel_lin > max_lin_vel:
            vel[:3] = vel[:3] * (max_lin_vel / vel_lin)
        vel_ang = np.linalg.norm(vel[3:])
        if vel_ang > max_ang_vel:
            vel[3:] = vel[3:] * (max_ang_vel / vel_ang)
        robot.SendCartesianMotionForce(
            pose.astype(float).tolist(), [0.0] * 6,
            vel.tolist(),
            max_lin_vel, max_ang_vel,
            max_lin_vel * 10.0, max_ang_vel * 10.0)
    else:
        robot.SendCartesianMotionForce(pose.astype(float).tolist(), [0.0] * 6)


def quat_to_mat(qwxyz):
    qw, qx, qy, qz = qwxyz
    R = np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
        [2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx)],
        [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy)],
    ])
    return R


def mat_to_quat(R):
    tr = np.trace(R)
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S
    q = np.array([qw, qx, qy, qz])
    return q / np.linalg.norm(q)


def clamp_pose_step(cur_pose, target_pose, max_step_m, max_rot_rad,
                    accel_m=0.0, accel_rot=0.0, prev_delta=None, prev_delta_rot=None):
    dp = target_pose[:3] - cur_pose[:3]
    d = np.linalg.norm(dp)
    if d > max_step_m:
        dp = dp * (max_step_m / d)

    if prev_delta is not None and accel_m > 0:
        d_delta = dp - prev_delta
        dd = np.linalg.norm(d_delta)
        if dd > accel_m:
            d_delta = d_delta * (accel_m / dd)
        dp = prev_delta + d_delta

    Rc = quat_to_mat(cur_pose[3:7])
    Rt = quat_to_mat(target_pose[3:7])
    dR = Rt @ Rc.T
    rotvec = matrix_to_rotvec(dR)
    angle = np.linalg.norm(rotvec)
    if angle > 1e-9 and angle > max_rot_rad:
        rotvec = rotvec * (max_rot_rad / angle)

    if prev_delta_rot is not None and accel_rot > 0:
        d_rv = rotvec - prev_delta_rot
        dr = np.linalg.norm(d_rv)
        if dr > accel_rot:
            d_rv = d_rv * (accel_rot / dr)
        rotvec = prev_delta_rot + d_rv

    R_new = rotvec_to_matrix(rotvec) @ Rc
    q_new = mat_to_quat(R_new)

    pose = np.empty(7)
    pose[:3] = cur_pose[:3] + dp
    pose[3:7] = q_new
    return pose


def matrix_to_rotvec(R):
    theta = np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    if theta < 1e-9:
        return np.zeros(3)
    axis = np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1],
    ]) / (2.0 * np.sin(theta))
    return axis * theta


def rotvec_to_matrix(rotvec):
    theta = np.linalg.norm(rotvec)
    if theta < 1e-9:
        return np.eye(3)
    axis = rotvec / theta
    K = np.array([
        [0, -axis[2], axis[1]],
        [axis[2], 0, -axis[0]],
        [-axis[1], axis[0], 0],
    ])
    return np.eye(3) + np.sin(theta) * K + (1.0 - np.cos(theta)) * (K @ K)


def map_vr_to_robot_pose(vr_pose, vr_ref, cur_tcp, pos_scale, rot_scale, axis_matrix=None):
    if axis_matrix is None:
        axis_matrix = VR_TO_ROBOT
    T_rel = np.linalg.inv(vr_ref) @ vr_pose
    dp_vr = T_rel[:3, 3]
    dR_vr = T_rel[:3, :3]
    dp = axis_matrix @ dp_vr * pos_scale
    dR = axis_matrix @ dR_vr @ axis_matrix.T
    dR_rotvec = matrix_to_rotvec(dR) * rot_scale
    dR_new = rotvec_to_matrix(dR_rotvec)
    target_pos = cur_tcp[:3] + dp
    target_rot = dR_new @ quat_to_mat(cur_tcp[3:7])
    target_q = mat_to_quat(target_rot)
    pose = np.empty(7)
    pose[:3] = target_pos
    pose[3:7] = target_q
    return pose


def map_vr_delta_pose(vr_cur, vr_prev, cur_tcp, pos_scale, rot_scale, axis_matrix=None):
    if axis_matrix is None:
        axis_matrix = VR_TO_ROBOT
    T_rel = np.linalg.inv(vr_prev) @ vr_cur
    dp_vr = T_rel[:3, 3]
    dR_vr = T_rel[:3, :3]
    dp = axis_matrix @ dp_vr * pos_scale
    dR = axis_matrix @ dR_vr @ axis_matrix.T
    dR_rotvec = matrix_to_rotvec(dR) * rot_scale
    dR_new = rotvec_to_matrix(dR_rotvec)
    target_pos = cur_tcp[:3] + dp
    target_rot = dR_new @ quat_to_mat(cur_tcp[3:7])
    target_q = mat_to_quat(target_rot)
    pose = np.empty(7)
    pose[:3] = target_pos
    pose[3:7] = target_q
    return pose


class VRVisualizer:

    def __init__(self, trail_len=100, tcp_ref=None):
        import matplotlib
        matplotlib.use("TkAgg")
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
        self.plt = plt
        self.Axes3D = Axes3D
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.ax.set_title("VR Teleop Live View")
        self.ax.set_xlabel("X (right)")
        self.ax.set_ylabel("Y (up)")
        self.ax.set_zlabel("Z (back)")
        self.ax.set_box_aspect((1, 1, 1))
        R = 1.0
        self.ax.set_xlim(-R, R)
        self.ax.set_ylim(-R, R)
        self.ax.set_zlim(-R, R)

        self.ax.scatter([0], [0], [0], color="k", s=50, marker="s", label="headset")

        self.sc = {
            "l": self.ax.scatter([], [], [], color="tab:blue", s=80, label="left"),
            "r": self.ax.scatter([], [], [], color="tab:red", s=80, label="right"),
        }
        self.trails = {
            "l": collections.deque(maxlen=trail_len),
            "r": collections.deque(maxlen=trail_len),
        }
        self.trail_plot = {
            "l": self.ax.plot([], [], [], color="tab:blue", alpha=0.4, lw=1)[0],
            "r": self.ax.plot([], [], [], color="tab:red", alpha=0.4, lw=1)[0],
        }
        self.axis_lines = {
            s: [self.ax.plot([], [], [], lw=2)[0] for _ in range(3)]
            for s in ("l", "r")
        }
        for s in ("l", "r"):
            for ln, c in zip(self.axis_lines[s], ["r", "g", "b"]):
                ln.set_color(c)
        self.tcp_sc = self.ax.scatter([], [], [], color="tab:green", s=120,
                                      marker="D", label="TCP target")
        self.tcp_base = np.asarray(tcp_ref, dtype=np.float64) if tcp_ref is not None else None

        self.ax.legend(loc="upper right")
        self.plt.ion()
        self.plt.show(block=False)
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def update(self, left_pose, right_pose, tcp_pose=None):
        for side, pose in (("l", left_pose), ("r", right_pose)):
            if pose is None:
                continue
            pos = pose[:3, 3].copy()
            R = pose[:3, :3]
            self.trails[side].append(pos)

            self.sc[side].set_offsets(np.atleast_2d(pos[:2]))
            self.sc[side].set_3d_properties([pos[2]], zdir="z")

            tr = np.array(self.trails[side]) if self.trails[side] else np.zeros((1, 3))
            self.trail_plot[side].set_data_3d(tr[:, 0], tr[:, 1], tr[:, 2])

            for ln, axis_vec in zip(self.axis_lines[side], np.eye(3)):
                tip = pos + R @ (axis_vec * 0.15)
                ln.set_data_3d([pos[0], tip[0]], [pos[1], tip[1]], [pos[2], tip[2]])

        if tcp_pose is not None:
            tcp_pos = np.asarray(tcp_pose[:3], dtype=np.float64)
            if self.tcp_base is not None:
                disp = tcp_pos - self.tcp_base
            else:
                disp = tcp_pos
            self.tcp_sc.set_offsets(np.atleast_2d(disp[:2]))
            self.tcp_sc.set_3d_properties([disp[2]], zdir="z")

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def close(self):
        try:
            self.plt.ioff()
            self.plt.close(self.fig)
        except Exception:
            pass


class TeleopLogger:
    def __init__(self, path):
        import csv
        self.csv = csv
        self.f = open(path, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow([
            "t", "frame",
            "hx", "hy", "hz", "hqw", "hqx", "hqy", "hqz",
            "b",
            "rx", "ry", "rz", "rqw", "rqx", "rqy", "rqz",
            "ax", "ay", "az", "aqw", "aqx", "aqy", "aqz",
            "lag_x", "lag_y", "lag_z",
        ])
        self.t0 = time.time()

    def log(self, frame, handle_pose, b, real_tcp, action):
        t = time.time() - self.t0
        h = handle_pose if handle_pose is not None else np.zeros(7)
        a = action if action is not None else np.zeros(7)
        lag = (a[:3] - real_tcp[:3]) * 1000.0 if (a is not None and real_tcp is not None) else [0, 0, 0]
        self.w.writerow([
            f"{t:.4f}", frame,
            *[f"{v:.5f}" for v in h[:3]],
            *[f"{v:.5f}" for v in h[3:7]],
            int(b),
            *[f"{v:.5f}" for v in real_tcp[:3]],
            *[f"{v:.5f}" for v in real_tcp[3:7]],
            *[f"{v:.5f}" for v in a[:3]],
            *[f"{v:.5f}" for v in a[3:7]],
            *[f"{v:.2f}" for v in lag],
        ])

    def close(self):
        try:
            self.f.close()
        except Exception:
            pass

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sn", default="Rizon4-062084")
    parser.add_argument("--scale", type=float, default=POS_SCALE,
                        help="hand->TCP position scale (default 0.30)")
    parser.add_argument("--rot-scale", type=float, default=ROT_SCALE,
                        help="hand->TCP rotation scale (default 0.30)")
    parser.add_argument("--no-rot-scale", action="store_true",
                        help="1:1 rotation mapping (rotation is NOT scaled; "
                             "overrides --rot-scale)")
    parser.add_argument("--max-speed", type=float, default=MAX_SPEED_MPS,
                        help="max TCP linear speed m/s (default 0.05)")
    parser.add_argument("--rot-speed", type=float, default=MAX_ROT_SPEED_RAD_S,
                        help="max TCP rotation speed rad/s (default 0.50)")
    parser.add_argument("--accel-mm", type=float, default=1.0,
                        help="per-frame position acceleration limit in mm "
                             "(smooths fast direction reversals; default 1.0)")
    parser.add_argument("--no-auto-scale", action="store_true",
                        help="disable adaptive position scale (handle speed "
                             "based scaling of the commanded TCP speed)")
    parser.add_argument("--handle-smooth", type=float, default=HANDLE_SMOOTH,
                        help="handle EMA smoothing 0..1 (default 0.5; lower = "
                             "smoother but more laggy, 1 = raw)")
    parser.add_argument("--handle-max-disp", type=float, default=HANDLE_MAX_DISP,
                        help="max handle displacement per frame in mm "
                             "(default 4.0; lower = smoother arm, less "
                             "responsive; higher = more responsive, rougher)")
    parser.add_argument("--axis-matrix-file", default="",
                        help="per-operator calibrated axis matrix JSON "
                             "(e.g. config/quest_axis_matrix.json)")
    parser.add_argument("--gripper", action="store_true",
                        help="Control the Robotiq gripper (right trigger closes, "
                             "release opens). Default: do NOT touch the gripper.")
    parser.add_argument("--visualize", action="store_true",
                        help="Show a live 3D window with controller motion and "
                             "the commanded TCP target.")
    parser.add_argument("--log", default="",
                        help="Write per-frame handle/state/action data to this "
                             "CSV file for offline analysis (high frequency).")
    parser.add_argument("--verbose-motion", action="store_true",
                        help="print every frame's target delta (debug)")
    parser.add_argument("--dry-run", action="store_true",
                        help="print targets without commanding the robot")
    args = parser.parse_args()

    pos_scale = max(0.05, min(1.0, args.scale))
    auto_scale = not args.no_auto_scale
    handle_smooth = max(0.05, min(1.0, args.handle_smooth))
    handle_max_disp = max(0.5, args.handle_max_disp) / 1000.0
    if args.no_rot_scale:
        rot_scale = 1.0  
    else:
        rot_scale = max(0.05, min(1.0, args.rot_scale))
    max_speed = max(0.005, min(10, args.max_speed))
    max_step_m = min(max_speed / FPS, 0.002)   # 2 mm/frame @ 60 Hz = 12 cm/s
    max_rot_speed = min(max(0.05, args.rot_speed), max_speed * 30.0)
    max_rot_step = min(max_rot_speed / FPS, 0.003)
    accel_m = max(0.0, args.accel_mm) / 1000.0
    accel_rot = 0.0015  # rad/frame^2
    axis_matrix = load_axis_matrix(args.axis_matrix_file)

    robot = None
    cur_pose = np.array([0.5, -0.45, 0.30, 1.0, 0.0, 0.0, 0.0])
    gripper = None

    print("=" * 60)
    print("  VR Teleop — Flexiv Rizon4")
    print("=" * 60)
    print("  Hold B (right ctrl) : move TCP (release = hold)")
    print("  Right trigger       : close gripper (release = open)")
    print("  Ctrl+C              : quit + return to start")
    print("  pos_scale={:.2f}  max_speed={:.3f} m/s  max_step={:.5f} m".format(
        pos_scale, max_speed, max_step_m))
    print("  rot_scale={:.2f}  max_rot_speed={:.3f} rad/s".format(
        rot_scale, max_rot_speed))
    if args.no_rot_scale:
        print("  rotation: 1:1 (NOT scaled)")

    if not args.dry_run:
        robot = flexivrdk.Robot(args.sn)
        print("[1/4] connected, fault={}".format(int(robot.fault())))
        if robot.fault():
            if not robot.ClearFault():
                print("  ERROR: cannot clear fault")
                return 1
            time.sleep(0.5)
        if not robot.estop_released():
            print("  ERROR: E-stop NOT released; aborting")
            return 3
        robot.Enable()
        wait_until_operational(robot)
        print("  robot enabled + operational")

        cur_pose = read_tcp(robot)
        print("[2/4] start tcp = {}".format(np.array2string(cur_pose, precision=4, floatmode="fixed")))
        if args.gripper:
            try:
                from gripper_modbus import RobotiqGripper
                gripper = RobotiqGripper("/dev/ttyACM0")
                gripper.open()
                print("  Robotiq gripper ready")
            except Exception as exc:
                print("  WARNING: gripper unavailable: {}".format(exc))
                gripper = None
        else:
            print("  --gripper not set; gripper control DISABLED")
    else:
        print("[1/4] DRY-RUN: robot connection skipped (virtual TCP)")

    print("[2/4] connecting to Quest VR...")
    vr = VRPoseReader()
    vr.start()
    wait_start = time.time()
    while True:
        lp_wait, rp_wait, _ = vr.get_state()
        if rp_wait is not None:
            break
        if time.time() - wait_start > 20.0:
            print("  No Quest pose data for 20s. Make sure:")
            print("   1. Quest is worn and the teleop app is in front of you")
            print("   2. Right controller is on and tracked")
            print("   3. adb devices shows the headset as 'device'")
            vr.stop()
            return 2
        time.sleep(0.5)
    print("  VR data received!")
    print("[4/4] READY. Hold B to move.\n")

    stop_event = threading.Event()

    def handle_sigint(sig, frame):
        stop_event.set()

    signal.signal(signal.SIGINT, handle_sigint)

    vr_ref = None
    tcp_ref = None
    prev_motion = None        # previous commanded target (feed-forward vel)
    prev_loop_t = None        # previous loop timestamp
    prev_b = False
    prev_trigger = False
    last_input_time = time.time()
    b_was_active = False
    frame_count = 0
    fps_timer = time.time()
    status_check_interval = max(1, int(round(FPS / 3)))  # ~3 Hz status checks
    last_status_check = 0
    tracking_ok = True
    visualizer = None
    if args.visualize:
        try:
            visualizer = VRVisualizer(trail_len=100, tcp_ref=cur_pose)
            print("  live visualization enabled")
        except Exception as exc:
            print("  WARNING: could not start visualization: {}".format(exc))
            visualizer = None
    logger = None
    if args.log:
        try:
            logger = TeleopLogger(args.log)
            print("  logging per-frame data to {}".format(args.log))
        except Exception as exc:
            print("  WARNING: could not start logger: {}".format(exc))
            logger = None
    if not args.dry_run:
        robot.SwitchMode(flexivrdk.Mode.NRT_CARTESIAN_MOTION_FORCE)
        time.sleep(0.05)

    try:
        while not stop_event.is_set():
            loop_start = time.time()
            lp, rp, btns = vr.get_state()
            b = bool(btns.get("B", False))
            trigger = bool(btns.get("RTr", False))

            now = time.time()
            data_fresh = (rp is not None) and (now - last_input_time <= INPUT_TIMEOUT_S)
            if rp is not None:
                last_input_time = now

            if b and not prev_b:
                vr_ref = rp.copy()
                if robot is not None:
                    cur_pose = read_tcp(robot)  # anchor on real TCP at grab start
                tcp_ref = cur_pose.copy()       # TCP reference for absolute map
                b_was_active = True
                print("  [TELEOP ON] B pressed; reference captured")
            if not b and prev_b:
                b_was_active = False
                print("  [TELEOP OFF] B released; holding")
            prev_b = b

            if gripper is not None and trigger != prev_trigger:
                if trigger:
                    gripper.close()
                else:
                    gripper.open()
            prev_trigger = trigger
            if not args.dry_run and robot is not None and (frame_count - last_status_check) >= status_check_interval:
                last_status_check = frame_count
                try:
                    real_tcp = read_tcp(robot)
                    drift = float(np.linalg.norm(real_tcp[:3] - cur_pose[:3]))
                    if drift > 0.02:
                        if tracking_ok:
                            print("  !!! tracking error {:.0f}mm; freezing target "
                                  "(arm likely at a limit/blocked). Release B and "
                                  "re-grab to re-anchor.".format(drift * 1000.0))
                        tracking_ok = False
                    else:
                        if not tracking_ok:
                            print("  tracking recovered ({:.0f}mm)".format(drift * 1000.0))
                        tracking_ok = True
                        cur_pose = real_tcp  # re-sync virtual target to real
                except Exception:
                    pass

            motion_target = None
            if b and rp is not None and vr_ref is not None and data_fresh and tracking_ok:
                tcp_target = np.empty(7)
                tcp_target[:3] = (tcp_ref[:3]
                                  + axis_matrix @ (rp[:3, 3] - vr_ref[:3, 3]) * pos_scale)
                R_vr_ref = vr_ref[:3, :3]
                R_rp = rp[:3, :3]
                R_vr_delta = R_rp @ R_vr_ref.T          # handle relative rot (world)
                R_robot_delta = axis_matrix @ R_vr_delta @ axis_matrix.T
                rv = matrix_to_rotvec(R_robot_delta) * rot_scale
                R_delta = rotvec_to_matrix(rv)
                R_tcp_ref = quat_to_mat(tcp_ref[3:7])
                R_target = R_delta @ R_tcp_ref
                tcp_target[3:7] = mat_to_quat(R_target)
                # per-frame step clamp (previous, simpler approach)
                motion_target = clamp_pose_step(
                    cur_pose, tcp_target, max_step_m, max_rot_step)
                cur_pose = motion_target.copy()
            else:
                motion_target = cur_pose

            if not args.dry_run and motion_target is not None:
                try:
                    if not robot.operational() or not robot.estop_released():
                        print("  !!! robot not operational / E-stop; holding !!!")
                    else:
                        # feed-forward velocity from the commanded target step
                        loop_now = time.time()
                        dt = (loop_now - prev_loop_t) if prev_loop_t is not None else 1.0 / FPS
                        if prev_motion is not None and dt > 1e-6 and b:
                            send_tcp_target(robot, motion_target, prev_motion, dt)
                        else:
                            send_tcp_target(robot, motion_target)
                        prev_motion = motion_target.copy()
                        prev_loop_t = loop_now
                        if args.verbose_motion and b:
                            print("  send target tcp = {}".format(
                                np.array2string(motion_target, precision=4,
                                                floatmode="fixed")))
                except Exception as exc:
                    print("  ERROR sending target: {}".format(exc))
            if logger is not None:
                try:
                    real_tcp_log = read_tcp(robot) if (not args.dry_run and robot is not None) else cur_pose
                    handle_pose_log = np.zeros(7)
                    if rp is not None:
                        handle_pose_log[:3] = rp[:3, 3]
                        handle_pose_log[3:7] = mat_to_quat(rp[:3, :3])
                    logger.log(frame_count, handle_pose_log, b, real_tcp_log,
                               motion_target if motion_target is not None else cur_pose)
                except Exception:
                    pass

            frame_count += 1
            now = time.time()
            if now - fps_timer > 5.0:
                fps = frame_count / (now - fps_timer)
                frame_count = 0
                fps_timer = now
                status = "ACTIVE" if b and data_fresh else "IDLE"
                print("  [{}] FPS: {:.1f}".format(status, fps))

            if visualizer is not None:
                try:
                    visualizer.update(lp, rp, cur_pose if not args.dry_run else motion_target)
                except Exception:
                    visualizer = None
                    print("  WARNING: visualization stopped")
            elapsed = time.time() - loop_start
            time.sleep(max(0, 1.0 / FPS - elapsed))

    except KeyboardInterrupt:
        pass
    finally:
        if logger is not None:
            logger.close()
            print("  teleop data log saved to {}".format(args.log))
        if visualizer is not None:
            visualizer.close()
        if not args.dry_run:
            try:
                print("\nReturning to start posture...")
                send_tcp_target(robot, cur_pose if cur_pose is not None else read_tcp(robot))
                time.sleep(0.3)
            except Exception:
                pass
            if gripper is not None:
                gripper.open()
        vr.stop()
        print("Done.")

if __name__ == "__main__":
    main()



