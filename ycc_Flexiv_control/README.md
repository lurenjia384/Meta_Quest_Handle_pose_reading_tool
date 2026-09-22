# ycc_Flexiv_control

Flexiv Rizon4 (SN: `Rizon4-062084`) + Robotiq 2F gripper 控制环境（基于 flexivrdk，无需 ROS）。

## 环境

- Conda 环境：`RoboTwin`（Python 3.10）
- flexivrdk: `1.9.0`（wheel 从 PyPI 下载，代理 `http://10.173.105.0:3129`）
- 依赖：numpy 1.26.4、scipy、pycryptodome 3.23.0（均已装好）

## 网络

- 机械臂控制器：`192.168.2.100`（Flexiv User Port 1 默认静态地址）
- 服务器网口：`eno2` = `192.168.2.2/24`（NetworkManager "有线连接 1"，manual，重启自动生效）
- 之前 eno2 误配的 `192.168.100.1/24` 网段是错的（那是机器人 General Port / 旧笔记本网段）

```bash
# 验证网络
ping 192.168.2.100
```

## 快速测试

```bash
conda activate RoboTwin
cd /home/robot/data_18T/ycc/VR/ycc_Flexiv_control

# 只读状态 + 极小关节运动（默认 0.001 rad ≈ 0.03% of ±π）+ 复位
python test_arm.py --sn Rizon4-062084
```

`test_arm.py` 参数：
- `--dq-rad 0.001`：关节1运动量（rad）
- `--max-vel 0.02` / `--max-acc 0.05`：速度/加速度限制（很低）
- `--no-return`：运动后不回到起始姿态
- 急停未释放时脚本会安全退出（返回码 3）

## 夹爪开合测试

Robotiq 2F（USB Modbus RTU，`/dev/ttyACM0`，device_id=9，baudrate=115200）：

```bash
conda activate RoboTwin
python gripper_test.py --device /dev/ttyACM0          # 1 次开合
python gripper_test.py --device /dev/ttyACM0 --cycles 3
```

参数：`--open-speed 48 --close-speed 96 --force 100 --timeout-s 15`。
脚本会激活（若未激活）→ 闭合 → 张开，每步打印 gACT/gOBJ/gPO 状态。

## 机械臂运动测试（J7 旋转 + TCP 位移）

```bash
conda activate RoboTwin
python arm_move_test.py --sn Rizon4-062084
```

默认：J7 旋转 `30%` 关节范围（J7 限位 ±2.967 rad，30% ≈ 1.78 rad），TCP 沿世界 Z 上移 `1cm`（0.01 m），完成后自动复位。
参数：`--j7-frac 0.30 --tcp-dz-m 0.01 --max-vel 0.10 --max-acc 0.20 --cart-vel 0.02 --cart-acc 0.05 --no-return`。
限位来自 `robot.info().q_min/q_max`，带 5° 安全边距。

> 注意：TCP 位移目标基于 **J7 旋转后的实际 TCP 位姿**（只改 z，保持当前朝向）。如果基于运动前的旧姿态，机械臂会同时把朝向转回去，产生倾斜的复合运动，看起来像"下移/乱动"。

## VR 手柄遥操作

```bash
conda activate RoboTwin
python vr_teleop.py --sn Rizon4-062084                    # 实机
python vr_teleop.py --dry-run                             # 只打印目标，不连机器人
python vr_teleop.py --axis-matrix-file config/quest_axis_matrix.json   # 加载标定矩阵
```

操作：
- 按住右手柄 **B**：TCP 跟随手柄位移/旋转（deadman，松开立即停）
- 右扳机 **RTr**：夹爪闭合（松开张开）
- Ctrl+C：退出并回到起始位姿

安全（默认保守）：
- 位移缩放 `0.30`（30cm 手柄 = 9cm TCP），旋转缩放 `0.30`
- 最大速度 `0.05 m/s`，单帧最大步长 `0.5mm`，旋转 `0.004 rad/帧`
- 输入超时 `0.15s`（数据卡住立即停）
- 工作空间硬边界 `WS_MIN/WS_MAX`
- 依赖 `gripper_modbus.py`（Robotiq 驱动）和 Quest teleop APK（adb logcat TAG `wE9ryARX`）

## 注意事项

- 运动前确认急停已释放（`robot.estop_released()`）
- flexivrdk 连接时**不要传** `network_interface_whitelist`（会触发 "All whitelist interfaces were filtered out" bug，不加即可自动发现）
- API 注意：1.9 版本用 `SendJointPosition(q, zero_vel, max_vel, max_acc)`（不是 SetJointPositions），`robot.mode()` 是方法不是属性
- 参考实现：`/home/robot/data_18T/ycc/flexiv-vr-control/`（别人的完整遥操作仓库，含 Robotiq Modbus 桥接、ROS bag 记录）
