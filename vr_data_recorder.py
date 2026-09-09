import os
import sys
import time
import csv
import signal
import subprocess
import threading
from datetime import datetime
import numpy as np
ADB_TAG = 'wE9ryARX'
MARKERS = {
    'A': ('RIGHT_LR', 'r'), 
    'B': ('RIGHT_FB', 'r'), 
    'X': ('LEFT_LR', 'l'),  
    'Y': ('LEFT_FB', 'l'), 
}
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
        side = pair[0]
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
            transforms[side] = transform
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

def pose_to_rpy(transform):
    x, y, z = transform[0, 3], transform[1, 3], transform[2, 3]
    R = transform[:3, :3]
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-6:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0.0
    return x, y, z, roll, pitch, yaw

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

def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else "vr_data.csv"
    print("=" * 64)
    print("  VR Controller Data Recorder")
    print("=" * 64)
    print("  Output: ", os.path.abspath(out_path))
    print()
    print("  Test phases (press the button to toggle the segment):")
    print("    A (right) : RIGHT hand  LEFT/RIGHT   motion")
    print("    B (right) : RIGHT hand  FORWARD/BACK  motion")
    print("    X (left)  : LEFT  hand  LEFT/RIGHT   motion")
    print("    Y (left)  : LEFT  hand  FORWARD/BACK  motion")
    print("    RTr       : right trigger (gripper test)")
    print("    LTr       : left  trigger (gripper test)")
    print("    Ctrl+C    : save and exit")
    print()

    vr = VRPoseReader()
    vr.start()
    print("Waiting for VR data...")
    while True:
        lp, rp, _ = vr.get_state()
        if lp is not None or rp is not None:
            break
        time.sleep(0.5)
    print("VR connected! Follow the workflow above, then Ctrl+C.")
    print()

    csv_file = open(out_path, 'w', newline='')
    writer = csv.writer(csv_file)
    writer.writerow([
        't', 'segment',
        'lx', 'ly', 'lz', 'lroll', 'lpitch', 'lyaw',
        'rx', 'ry', 'rz', 'rroll', 'rpitch', 'ryaw',
        'RG', 'LG', 'RTr', 'LTr', 'A', 'B', 'X', 'Y', 'all_buttons',
    ])

    active_segments = set()
    prev_buttons = {}

    t0 = time.time()
    try:
        while True:
            lp, rp, btns = vr.get_state()
            for btn, (name, side) in MARKERS.items():
                pressed = btns.get(btn, False)
                if pressed and not prev_buttons.get(btn, False):
                    if name in active_segments:
                        active_segments.discard(name)
                        print(f"  [segment OFF] {name}")
                    else:
                        active_segments.add(name)
                        print(f"  [segment ON ] {name}")
            prev_buttons = btns

            segment = '+'.join(sorted(active_segments)) if active_segments else ''

            lvals = [0.0] * 6
            rvals = [0.0] * 6
            if lp is not None:
                lvals = pose_to_rpy(lp)
            if rp is not None:
                rvals = pose_to_rpy(rp)
            btn_str = ','.join(k for k, v in btns.items() if v is True and isinstance(v, bool))
            writer.writerow([
                f"{time.time() - t0:.3f}", segment,
                *[f"{v:.4f}" for v in lvals],
                *[f"{v:.4f}" for v in rvals],
                int(btns.get('RG', False)), int(btns.get('LG', False)),
                int(btns.get('RTr', False)), int(btns.get('LTr', False)),
                int(btns.get('A', False)), int(btns.get('B', False)),
                int(btns.get('X', False)), int(btns.get('Y', False)),
                btn_str,
            ])
            csv_file.flush()
            lx, ly, lz = lvals[0], lvals[1], lvals[2]
            rx, ry, rz = rvals[0], rvals[1], rvals[2]
            grip = f"RTr={btns.get('RTr',False)} LTr={btns.get('LTr',False)}"
            print(f"\r  t={time.time()-t0:5.1f}s seg={segment:<14} "
                  f"L=({lx:+.3f},{ly:+.3f},{lz:+.3f}) "
                  f"R=({rx:+.3f},{ry:+.3f},{rz:+.3f}) {grip}   ", end='', flush=True)

            time.sleep(1.0 / 60)

    except KeyboardInterrupt:
        print("\n\nStopping...")
    finally:
        csv_file.close()
        vr.stop()
        print(f"Data saved to {os.path.abspath(out_path)}")
        print("Done.")


if __name__ == '__main__':
    main()
