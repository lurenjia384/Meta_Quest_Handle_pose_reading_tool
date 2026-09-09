# Quest VR 手柄位姿读取工具

读取 Meta Quest 2/3 VR 头显两个手柄（左/右）的实时位姿数据（位置 + 姿态）。

## 系统要求

- Ubuntu 20.04+ (其他 Linux 发行版需自行调整包管理器命令)
- Python 3.8+
- Meta Quest 2 或 Quest 3 头显
- USB-C 数据线

## 安装步骤

### 1. 安装系统依赖

```bash
sudo apt update
sudo apt install adb
```

### 2. 安装 Python 依赖

```bash
pip install pure-python-adb numpy
```

### 3. Quest 头显设置

#### 3.1 开启开发者模式（能直接第三步就第三步）

1. 访问 [Meta 开发者平台](https://developer.oculus.com/)，用 Meta 账号登录并创建组织（名称随意），绑定信用卡完成验证
2. 打开手机端 Meta Quest App → 设备设置 → 开发者模式 → 开启开关
3. 在头显内进入 **设置 → 系统 → 开发者选项** → 开启 "未知来源" 权限

#### 3.2 设置头显休眠时长

进入头显 **设置 → 常规 → 电源** → 将 "显示屏关闭时间" 调成 **4 小时**（防止息屏导致数据中断）

### 4. 连接 Quest 与电脑

1. 用 USB-C 数据线连接 Quest 与电脑
2. 戴上 Quest 头显，弹出 **"允许 USB 调试"** 提示时：
   - 勾选 **"始终允许来自此计算机"**
   - 点击 **"允许"**
3. 验证连接：
   ```bash
   adb devices
   ```
   应显示类似 `1WMHHA66CB2053  device`（状态为 `device` 而非 `unauthorized`）

### 5. 安装 teleop APK

APK 文件从 GitHub 下载：

```bash
wget -O teleop-debug.apk \
  "https://raw.githubusercontent.com/guardstrikelab/questVR/main/src/oculus_reader/APK/teleop-debug.apk"
```
安装 APK 文件
```bash
adb install teleop-debug.apk
```

## 使用方法

```bash
python3 get_vr_pose.py
```

脚本输出内容包括：
- **位置 (Position)**：X, Y, Z（单位：米）
- **姿态 (Orientation)**：Roll, Pitch, Yaw（弧度和角度）
- **4x4 变换矩阵**
- **按钮状态**

按 `Ctrl+C` 停止。

---

## Python 环境与依赖


### 环境安装指令（新机器参考）

```bash
pip install mujoco numpy scipy

# 2. 真机环境（lerobot，python 3.11）
pip install "lerobot[so101]" placo numpy scipy
```

系统依赖：

```bash
sudo apt update
sudo apt install adb
```

---

## 遥操作脚本用法

### 1. 仿真遥操作（双臂可视化）— `sim_teleop.py`

```bash
python sim_teleop.py                          # 默认 scale=0.6, smooth=0.6
python sim_teleop.py --scale 0.4 --smooth 0.8 # 自定义缩放和平滑
python sim_teleop.py --help                   # 查看全部参数
```

**参数**：

| 参数 | 含义 | 默认 |
|---|---|---|
| `--scale` | 末端运动缩放（手柄位移 → 臂位移比例） | 0.6 |
| `--smooth` | 平滑滤波比例 0~1（越高越跟手，越低越平滑） | 0.6 |
| `--rot-step` | 每帧目标旋转最大变化（弧度，防 360° 跳变） | 0.3 |

**操作**：按住**右手 Grip** 遥操作（松开暂停）→ 移动手柄控制双臂；**左手 Grip** 按一次复位；**左右扳机**控制夹爪闭合。

### 2. 真机遥操作（单臂）— `real_teleop.py`

```bash
python real_teleop.py                          # 试运行（只打印，不动电机）
python real_teleop.py --execute                # 真机执行
python real_teleop.py --execute --scale 0.5 --smooth 0.8  # 自定义参数
python real_teleop.py --gripper-test           # 夹爪开合自检（不连 VR）
```

**参数**：

| 参数 | 含义 | 默认 |
|---|---|---|
| `--port` | 电机串口 | `/dev/ttyACM0` |
| `--execute` | 发送动作到电机（默认试运行） | 关闭 |
| `--scale` | 末端运动缩放 | 0.6 |
| `--smooth` | 关节限速平滑 0~1（1.0=满速，越低越平滑） | 1.0 |
| `--gripper-test` | 夹爪自检模式 | 关闭 |

**操作**：按住**右手 Grip** 遥操作；**左手 Grip** 复位回 home；**右手扳机**闭合夹爪。

> **首次上电建议**：先运行 `python go_home.py` 让机械臂平滑回到 home 位姿，再开始遥操作，避免从异常姿态直接跳变导致堵转。

### 3. 手柄数据记录（坐标分析）— `vr_data_recorder.py`

```bash
python vr_data_recorder.py            # 记录到 vr_data.csv
python vr_data_recorder.py data.csv   # 指定输出文件
```

按 `A/B/X/Y` 分段标记左右手移动，`RTr/LTr` 测试扳机，`Ctrl+C` 保存。

### 4. 真机调试工具

```bash
python go_home.py --dry-run            # 查看回 home 计划
python go_home.py                      # 平滑回 home（1°/步）
python debug_robot.py --no-move        # 只读取关节状态
python debug_robot.py --step 15        # 逐关节 ±15° 测试
python detect_direction.py --joint all # 3° 小步检测电机方向
```

---

## 坐标系约定（实测）

通过 `get_vr_pose.py` 实测本机 Quest 手柄坐标系：

- **X**：右 +，左 -
- **Y**：上 +，下 -
- **Z**：后 +，前 -（即手柄在面前时 Z 为负）
- 绕主轴（手柄前向轴）：顺时针 yaw 为 -，逆时针为 +

机器人坐标系（MuJoCo / SO-101）：X=前、Y=左、Z=上。映射矩阵 `VR_TO_ROBOT`（det=+1 纯旋转）：

```
Robot X = -VR Z   （手柄前推 → 臂向前）
Robot Y = -VR X   （手柄右移 → 臂向右）
Robot Z = +VR Y   （手柄上抬 → 臂向上）
```
