# Quest VR controller pose reading tool

Read real-time pose data (position+posture) of the two handles (left/right) of the Meta Quest 2/3 VR headset.

## System requirements

- Ubuntu 20.04+(other Linux distributions require self adjustment of package manager commands)
- Python 3.8+
- Meta Quest 2 or Quest 3 headset
- USB-C data cable

## Installation steps

### 1. Install system dependencies

```bash
sudo apt update
sudo apt install adb
```

### 2. Install Python dependencies

```bash
pip install pure-python-adb numpy
```

### 3. Quest headset settings

#### 3.1 Enable developer mode (if possible, proceed to the third step directly)

1. Visit the Meta Developer Platform（https://developer.oculus.com/）Log in with a Meta account and create an organization (name arbitrary), bind a credit card to complete verification
2. Open the Meta Quest App on your mobile phone ->Device Settings ->Developer Mode ->Turn On Switch
3. Go to **Settings → System → Developer Options** → Enable "Unknown Source" permission in the headset

#### 3.2 Setting the sleep duration of the headset

Go to the headset **settings → general → power** → set the "display screen off time" to **4 hours** (to prevent data interruption caused by screen shutdown)

### 4. Connect Quest to the computer

1. Connect Quest to the computer using a USB-C data cable
2. When wearing the Quest headset and the **"Allow USB debugging"** prompt pops up:
- Check **"Always allow from this computer"**
- Click **"Allow"**
3. Verify connection:
```bash
adb devices
```
Should display a device similar to '1WMHHA66CB2053 device' (with a status of 'device' instead of 'unauthorized')

### 5. Install teleop APK

Download APK file from GitHub:

```bash
wget -O teleop-debug.apk \
"https://raw.githubusercontent.com/guardstrikelab/questVR/main/src/oculus_reader/APK/teleop-debug.apk"
```
Install APK file
```bash
adb install teleop-debug.apk
```

##Usage method

```bash
python3 get_vr_pose.py
```

The script output includes:
- **Position**: X, Y, Z (unit: meter)
- **Orientation**: Roll, Pitch, Yaw (radians and angles)
- **4x4 transformation matrix**
- **Button status**

Press Ctrl+C to stop.

---

## Python environments & dependencies

### Install commands (fresh machine reference)

```bash
conda activate RoboTwin
pip install mujoco numpy scipy

# 2. Real-robot env (lerobot, python 3.11)
pip install "lerobot[so101]" placo numpy scipy
```

System dependency:

```bash
sudo apt update
sudo apt install adb
```

---

## Teleoperation script usage

### 1. Simulation teleop (dual-arm visualization) — `sim_teleop.py`

```bash
python sim_teleop.py                          # defaults scale=0.6, smooth=0.6
python sim_teleop.py --scale 0.4 --smooth 0.8 # custom scale & smoothing
python sim_teleop.py --help                   # all options
```

**Options**:

| Arg | Meaning | Default |
|---|---|---|
| `--scale` | End-effector motion scale (hand displacement → arm displacement) | 0.6 |
| `--smooth` | Smoothing ratio 0..1 (higher = more responsive, lower = smoother) | 0.6 |
| `--rot-step` | Max target rotation change per frame (rad, prevents 360° wraps) | 0.3 |

**Controls**: HOLD **Right Grip** to teleop (release = pause) and move the handle to drive both arms; press **Left Grip** once to reset; **Left/Right triggers** close the grippers.

### 2. Real-robot teleop (single arm) — `real_teleop.py`

```bash
python real_teleop.py                          # dry-run (print only, no motion)
python real_teleop.py --execute                # execute on the real robot
python real_teleop.py --execute --scale 0.5 --smooth 0.8  # custom params
python real_teleop.py --gripper-test           # gripper open/close self-test (no VR)
```

**Options**:

| Arg | Meaning | Default |
|---|---|---|
| `--port` | Motor serial port | `/dev/ttyACM0` |
| `--execute` | Send actions to motors (default dry-run) | off |
| `--scale` | End-effector motion scale | 0.6 |
| `--smooth` | Joint rate-limit smoothing 0..1 (1.0 = full speed, lower = smoother) | 1.0 |
| `--gripper-test` | Gripper self-test mode | off |

**Controls**: HOLD **Right Grip** to teleop; press **Left Grip** to reset to home; **Right trigger** closes the gripper.

> **First power-on**: run `python go_home.py` first to smoothly move the arm to the home pose before teleop, avoiding stalls from jumping to a far target from an abnormal pose.

### 3. VR data recorder (coordinate analysis) — `vr_data_recorder.py`

```bash
python vr_data_recorder.py            # records to vr_data.csv
python vr_data_recorder.py data.csv   # custom output file
```

Press `A/B/X/Y` to segment left/right hand motions, `RTr/LTr` to test triggers, `Ctrl+C` to save.

### 4. Real-robot debug tools

```bash
python go_home.py --dry-run            # preview home-return plan
python go_home.py                      # smooth return to home (1°/step)
python debug_robot.py --no-move        # read joint states only
python debug_robot.py --step 15        # per-joint ±15° test
python detect_direction.py --joint all # 3° small-step motor direction check
```

---

## Coordinate convention (measured)

Measured on this Quest via `get_vr_pose.py`:

- **X**: right +, left -
- **Y**: up +, down -
- **Z**: back +, forward - (handle in front of you ⇒ negative Z)
- Yaw around the main (handle forward) axis: clockwise -, counter-clockwise +

Robot frame (MuJoCo / SO-101): X=forward, Y=left, Z=up. Mapping `VR_TO_ROBOT` (det=+1 proper rotation):

```
Robot X = -VR Z   (handle push forward  → arm forward)
Robot Y = -VR X   (handle move right    → arm right)
Robot Z = +VR Y   (handle move up       → arm up)
```
