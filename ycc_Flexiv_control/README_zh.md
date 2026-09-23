# ycc_Flexiv_control

Flexiv Rizon4（SN：`Rizon4-062084`）+ Robotiq 2F 夹爪控制环境（基于 `flexivrdk`，无需 ROS）。包含 Quest VR 遥操作、高频流式控制，以及输出标准 LeRobot v3.0 数据集的双相机数据采集。

## 环境

- flexivrdk：`1.9.0`（wheel 从 PyPI 下载）
- 其他依赖：numpy 1.26.4、scipy、pycryptodome 3.23.0、opencv-python、matplotlib、以及lerobot相关包

> 两个环境刻意分离：`flexivrdk` 与 `lerobot` 未装在一起。采集脚本在
> RoboTwin 环境写 RAW 文件，再由 `convert_to_lerobot.py` 在 lerobot 环境
> 生成数据集。

## 网络

- 机械臂控制器：`192.168.2.100`（Flexiv User Port 1 默认静态地址）
- 服务器网口：`eno2` = `192.168.2.2/24`（NetworkManager "有线连接 1"，manual）

```bash
ping 192.168.2.100
```

## 快速测试

```bash
cd /home/robot/data_18T/ycc/VR/ycc_Flexiv_control

python test_arm.py --sn Rizon4-062084          # 读状态 + 极小关节运动 + 复位
python gripper_test.py --device /dev/ttyACM0    # 1 次开合
python arm_move_test.py --sn Rizon4-062084      # J7 旋转 + TCP 位移测试

```

## VR 遥操作（`vr_teleop.py`）

基于 Quest 手柄的高频流式笛卡尔控制。

```bash

python vr_teleop.py --scale 0.3 --max-speed 0.8 --no-rot-scale --gripper \
    --visualize --log teleop_session.csv
```

操作：
- 按住右手柄 **B**：遥操作（clutch）。手柄位移/旋转相对 B 按下时的参考
  映射到 TCP（绝对相对映射）。
- 右扳机 **RTr**：夹爪闭合（松开张开）。
- 左手柄 **X / Y**：预留作数据采集控制（见下文采集器）。
- Ctrl+C：退出并回到起始位姿。

参数：
- `--scale`：手柄→TCP 位置缩放（默认 0.3）
- `--max-speed`：最大 TCP 线速度 m/s（默认 0.8）
- `--no-rot-scale`：旋转 1:1 映射
- `--rot-speed`：最大旋转速度 rad/s（默认 0.5）
- `--accel-mm`：每帧位置加速度限制 mm（默认 1.0）
- `--handle-smooth`：手柄 EMA 平滑 0..1（默认 0.5）
- `--gripper`：控制 Robotiq 夹爪
- `--visualize`：实时 3D 窗口（手柄 + TCP）
- `--log FILE`：逐帧 CSV 日志（手柄 / 真实 state / action）

控制架构（依据 Flexiv_Rizon4_Quest2_VR 优化方案文档）：
- **绝对 clutch 映射**：B 按下记录 `vr_ref` + `tcp_ref`；目标 =
  `tcp_ref + axis_matrix @ (vr_now - vr_ref) * scale`。旋转用相对旋转矩阵
  （世界系共轭），不用逐轴欧拉角。
- **参考限速器（governor）**：速度/加速度连续限制，固定 100 Hz 内部周期；
  机械臂不会被要求"冲刺"追赶原始 VR 跳变。
- **高频流式**：伺服循环每个 tick 都发送目标（`FPS` 默认 100 Hz），与 VR
  数据新鲜度无关；VR 数据过期时经 governor 平滑减速。
- 前馈速度使用 governor 的 `v_cmd` 传给 `SendCartesianMotionForce`
  （含 velocity + max_linear_vel/acc）。

`vr_teleop_bak.py` 是早期基于 clamp 版本的备份。

## 数据采集（`record_lerobot_dataset.py`）

遥操作同时记录双彩色相机 + Flexiv TCP state/action。写 RAW 文件
（PNG + CSV）——在 RoboTwin 环境运行。

```bash

python record_lerobot_dataset.py --scale 0.3 --max-speed 0.8 --no-rot-scale \
    --gripper --session-dir ./recordings/session1
```

操作：
- 按住 **B**：遥操作（与 vr_teleop 相同）
- **X**（左手柄）：开始记录一个 episode
- **Y**（左手柄）：结束并保存当前 episode
- Ctrl+C：退出（会保存未关闭的 episode）

相机（两个独立 RealSense 彩色流）：
- **D435** 彩色 → `/dev/video4`（设备索引 4）
- **D435i** 彩色 → `/dev/video10`（设备索引 10）
- 均为 640×480；双相机共享 USB 带宽 → 采集帧率 15。
  `dual_camera.py` 显式用 `CAP_V4L2`（默认后端可能挂到 depth/元数据节点，
  读不到帧）。

state / action（各 8 维）：
- `x, y, z` TCP 位置（m），`qw, qx, qy, qz` TCP 四元数，`gripper` 夹爪
- state = 真实 TCP + 夹爪；action = 发送的目标 + 夹爪

每个 episode 的输出布局：
```
recordings/session1/
├── episode_0/
│   ├── data.csv              # timestamp, frame, s_*, a_*
│   └── images/
│       ├── top_00000.png     # D435 彩色
│       └── front_00000.png   # D435i 彩色
├── episode_1/ ...
```

## 转成 LeRobot v3.0（`convert_to_lerobot.py`）

生成标准 LeRobot 数据集（codebase v3.0），图像存为 videos/ MP4。

```bash
conda activate lerobot
python convert_to_lerobot.py --session-dir ./recordings/session1 \
    --dataset-root ./datasets --dataset-name flexiv_vr_001 \
    --task pick_and_place
```

数据集布局（标准 v3.0）：
```
datasets/flexiv_vr_001/
├── data/chunk-000/file-000.parquet      # 标量 state/action/时间戳
├── videos/observation.image.top/*.mp4   # SVT-AV1 编码
├── videos/observation.image.front/*.mp4
└── meta/ (info.json, episodes, tasks, stats)
```

注意：
- `info.json` 显示 `codebase_version: "v3.0"`、`robot_type: flexiv_rizon4`。
- 图像特征必须用 `dtype="video"` 才能存为 MP4（标准 v3.0 布局）；
  `dtype="image"` 会把 bytes 存进 parquet。
- 特征 shape 必须传 tuple（lerobot 0.4.4 校验 np.shape(tuple) != list，
  传 list 会误报不匹配）。
- `--dataset-name` 目录不能已存在。
- 需要 ffmpeg 且带 `libsvtav1`（本机已装）。

## 调试 / 测试工具

```bash
python debug_tcp_tracking.py --duration 8 --amplitude 0.04   # TCP 跟踪滞后测量
python sim_vr_teleop_test.py --pattern smooth                # 合成 VR 仿真
python vr_visualize.py                                       # 手柄实时 3D 显示
```

## 注意事项

- 运动前确认急停已释放（`robot.estop_released()`）。
- flexivrdk 1.9 API：`SendJointPosition(q, zero_vel, max_vel, max_acc)`，
  `robot.mode()` 是方法不是属性。
- Quest teleop APK 需在运行（adb logcat TAG `wE9ryARX`）。
- 参考实现：`/home/robot/data_18T/ycc/flexiv-vr-control/`
- **复位： python reset.py --robot-sn Rizon4-062084**！！！
- **复位： python reset.py --robot-sn Rizon4-062084**！！！
- **复位： python reset.py --robot-sn Rizon4-062084**！！！