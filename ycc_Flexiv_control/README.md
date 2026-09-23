# ycc_Flexiv_control

Flexiv Rizon4 (SN: `Rizon4-062084`) + Robotiq 2F gripper control environment
(based on `flexivrdk`, no ROS). Includes Quest VR teleoperation, high-frequency
streaming control, and a dual-camera data recorder that outputs standard
LeRobot v3.0 datasets.

## Environment

- flexivrdk: `1.9.0` 
- Other dependencies: numpy 1.26.4, scipy, pycrypto dome 3.23.0, opencv python, matplotlib, and related packages for lerobot


## Network

- Robot controller: `192.168.2.100` (Flexiv User Port 1 default static)
- PC NIC: `eno2` = `192.168.2.2/24` (NetworkManager "有线连接 1", manual)

```bash
ping 192.168.2.100
```

## Quick checks

```bash

cd VR/ycc_Flexiv_control

python test_arm.py --sn Rizon4-062084          # read state + tiny joint move + return
python gripper_test.py --device /dev/ttyACM0    # 1 open/close cycle
python arm_move_test.py --sn Rizon4-062084      # J7 rotate + TCP move test
```

## VR teleoperation (`vr_teleop.py`)

High-frequency streaming Cartesian control from the Quest controllers.

```bash

python vr_teleop.py --scale 0.3 --max-speed 0.8 --no-rot-scale --gripper \
    --visualize --log teleop_session.csv
```

Controls:
- Hold right **B** : teleop active (clutch). Handle displacement/rotation maps
  to TCP displacement from the B-press reference (absolute mapping).
- Right trigger **RTr** : close gripper (release opens).
- Left **X** / **Y** : reserved for data recording (see recorder below).
- Ctrl+C : quit and return to start posture.

Options:
- `--scale` : hand→TCP position scale (default 0.3)
- `--max-speed` : max TCP linear speed m/s (default 0.8)
- `--no-rot-scale` : 1:1 rotation mapping
- `--rot-speed` : max rotation speed rad/s (default 0.5)
- `--accel-mm` : per-frame position acceleration limit mm (default 1.0)
- `--handle-smooth` : handle EMA smoothing 0..1 (default 0.5)
- `--gripper` : control the Robotiq gripper
- `--visualize` : live 3D window (controllers + TCP)
- `--log FILE` : per-frame CSV log (handle / real state / action)

Control architecture (per the Flexiv_Rizon4_Quest2_VR optimization doc):
- **Absolute clutch mapping**: on B press record `vr_ref` + `tcp_ref`; target =
  `tcp_ref + axis_matrix @ (vr_now - vr_ref) * scale`. Rotation via relative
  rotation matrix (world-frame conjugation), not per-axis Euler.
- **Reference governor**: velocity/acceleration-limited, fixed 100 Hz internal
  period; the arm is never asked to sprint after raw VR jumps.
- **High-frequency streaming**: servo loop always sends a target every tick at
  `FPS` (default 100 Hz), independent of VR data freshness; stale VR decelerates
  smoothly via the governor.
- Feed-forward velocity from the governor `v_cmd` is passed to
  `SendCartesianMotionForce` (velocity + max_linear_vel/acc).

`vr_teleop_bak.py` is a backup of the earlier clamp-based version.

## Data recording (`record_lerobot_dataset.py`)

Records dual color cameras + Flexiv TCP state/action while teleoperating.
Writes RAW files (PNG + CSV) — run in the RoboTwin env.

```bash

python record_lerobot_dataset.py --scale 0.3 --max-speed 0.8 --no-rot-scale \
    --gripper --session-dir ./recordings/session1
```

Controls:
- Hold **B** : teleop (same as vr_teleop)
- **X** (left) : START recording an episode
- **Y** (left) : STOP and close the episode
- Ctrl+C : quit (saves the open episode)

Cameras (two independent RealSense color streams):
- **D435** color → `/dev/video4` (device index 4)
- **D435i** color → `/dev/video10` (device index 10)
- Both at 640×480; two cameras share USB bandwidth → record FPS 15.
  `dual_camera.py` uses explicit `CAP_V4L2` (the default backend can attach to
  a depth/metadata node and return no frames).

State / action (8-dim each):
- `x, y, z` TCP position (m), `qw, qx, qy, qz` TCP quaternion, `gripper`
- state = real TCP + gripper; action = commanded target + gripper

Output layout per episode:
```
recordings/session1/
├── episode_0/
│   ├── data.csv              # timestamp, frame, s_*, a_*
│   └── images/
│       ├── top_00000.png     # D435 color
│       └── front_00000.png   # D435i color
├── episode_1/ ...
```

## Convert to LeRobot v3.0 (`convert_to_lerobot.py`)

Builds a standard LeRobot dataset (codebase v3.0) with videos/ MP4 files.

```bash
conda activate lerobot
python convert_to_lerobot.py --session-dir ./recordings/session1 \
    --dataset-root ./datasets --dataset-name flexiv_vr_001 \
    --task pick_and_place
```

Dataset layout (standard v3.0):
```
datasets/flexiv_vr_001/
├── data/chunk-000/file-000.parquet      # scalar state/action/timestamps
├── videos/observation.image.top/*.mp4   # SVT-AV1 encoded
├── videos/observation.image.front/*.mp4
└── meta/ (info.json, episodes, tasks, stats)
```

Notes:
- `info.json` shows `codebase_version: "v3.0"` and `robot_type: flexiv_rizon4`.
- Image features must use `dtype="video"` so frames are stored as MP4 (the
  standard v3.0 layout); `dtype="image"` stores bytes inside parquet instead.
- Feature shapes must be passed as tuples (lerobot 0.4.4 validates
  np.shape(tuple) != list and would reject a list).
- `--dataset-name` dir must not already exist.
- Requires `ffmpeg` with `libsvtav1` (present on this machine).

## Debug / test tools

```bash
python debug_tcp_tracking.py --duration 8 --amplitude 0.04   # TCP tracking lag
python sim_vr_teleop_test.py --pattern smooth                # synthetic VR sim
python vr_visualize.py                                       # live controller 3D view
```

## Notes

- Verify e-stop released before motion (`robot.estop_released()`).
- flexivrdk 1.9 API: `SendJointPosition(q, zero_vel, max_vel, max_acc)`,
  `robot.mode()` is a method (not a property).
- Quest teleop APK must be running (adb logcat TAG `wE9ryARX`).
- Reference implementation: `/home/robot/data_18T/ycc/flexiv-vr-control/`
- **Reset: python reset.py --robot-sn Rizon4-062084**
