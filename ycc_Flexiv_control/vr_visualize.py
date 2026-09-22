import sys
import time
import threading
import subprocess
import argparse
import collections

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

TAG = "wE9ryARX"
APP_COMPONENT = "com.rail.oculus.teleop/com.rail.oculus.teleop.MainActivity"

TRAIL_LEN = 100  
AXIS_LEN = 0.15 


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
        side = pair[0]
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
            transforms[side] = transform
    return transforms, buttons_string


class VRPoseReader:
    def __init__(self):
        self._lock = threading.Lock()
        self.left_pose = None
        self.right_pose = None
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
            try:
                line = raw.decode("utf-8", errors="replace").strip()
            except Exception:
                continue
            if TAG not in line:
                continue
            try:
                data = line.split(TAG + ": ")[1]
                transforms, _ = parse_transforms(data)
                if transforms is None:
                    continue
                with self._lock:
                    if "l" in transforms:
                        self.left_pose = transforms["l"].copy()
                    if "r" in transforms:
                        self.right_pose = transforms["r"].copy()
            except Exception:
                pass

    def get_poses(self):
        with self._lock:
            return (
                self.left_pose.copy() if self.left_pose is not None else None,
                self.right_pose.copy() if self.right_pose is not None else None,
            )

    def stop(self):
        self._running = False
        if self._process:
            self._process.terminate()
        if self._thread:
            self._thread.join(timeout=2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headset-pos", nargs=3, type=float,
                        default=[0.0, 0.0, 0.0],
                        help="headset origin in world (default 0 0 0)")
    parser.add_argument("--trail", type=int, default=TRAIL_LEN,
                        help="trail length in samples (default 100)")
    args = parser.parse_args()

    print("Connecting to Quest VR...")
    vr = VRPoseReader()
    vr.start()
    print("Waiting for pose data...")

    # wait for first data
    while True:
        lp, rp = vr.get_poses()
        if lp is not None or rp is not None:
            break
        time.sleep(0.5)
    print("VR data received! Drawing... (Ctrl+C to quit)")

    trails = {
        "l": collections.deque(maxlen=args.trail),
        "r": collections.deque(maxlen=args.trail),
    }
    fig = plt.figure(figsize=(9, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_title("Quest VR Controllers (live)")
    ax.set_xlabel("X (right)")
    ax.set_ylabel("Y (up)")
    ax.set_zlabel("Z (back)")
    ax.set_box_aspect((1, 1, 1))
    RANGE_M = 1.0
    ax.set_xlim(-RANGE_M, RANGE_M)
    ax.set_ylim(-RANGE_M, RANGE_M)
    ax.set_zlim(-RANGE_M, RANGE_M)

    head_origin = np.array(args.headset_pos)
    ax.scatter(*head_origin, color="k", s=60, marker="s", label="headset")
    sc = {
        "l": ax.scatter([], [], [], color="tab:blue", s=80, label="left"),
        "r": ax.scatter([], [], [], color="tab:red", s=80, label="right"),
    }
    trails_plot = {
        "l": ax.plot([], [], [], color="tab:blue", alpha=0.4, lw=1)[0],
        "r": ax.plot([], [], [], color="tab:red", alpha=0.4, lw=1)[0],
    }
    axis_lines = {s: [ax.plot([], [], [], lw=2)[0] for _ in range(3)]
                  for s in ("l", "r")}
    axis_colors = ["r", "g", "b"]
    for s in ("l", "r"):
        for ln, c in zip(axis_lines[s], axis_colors):
            ln.set_color(c)

    ax.legend(loc="upper right")
    plt.ion()
    plt.show(block=False)
    fig.canvas.draw_idle()
    fig.canvas.flush_events()

    frame = 0
    try:
        while True:
            lp, rp = vr.get_poses()
            for side, pose in (("l", lp), ("r", rp)):
                if pose is None:
                    continue
                pos = pose[:3, 3].copy()
                R = pose[:3, :3]
                trails[side].append(pos)
                sc[side].set_offsets(np.atleast_2d(pos[:2]))
                sc[side].set_3d_properties([pos[2]], zdir="z")
                tr = np.array(trails[side]) if trails[side] else np.zeros((1, 3))
                trails_plot[side].set_data_3d(tr[:, 0], tr[:, 1], tr[:, 2])
                for ln, axis_vec in zip(axis_lines[side], np.eye(3)):
                    tip = pos + R @ (axis_vec * AXIS_LEN)
                    ln.set_data_3d([pos[0], tip[0]], [pos[1], tip[1]], [pos[2], tip[2]])

            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            time.sleep(1.0 / 30.0)
            frame += 1

    except KeyboardInterrupt:
        pass
    finally:
        vr.stop()
        plt.ioff()
        plt.close(fig)
        print("Done.")


if __name__ == "__main__":
    main()
