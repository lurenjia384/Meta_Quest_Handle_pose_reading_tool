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
