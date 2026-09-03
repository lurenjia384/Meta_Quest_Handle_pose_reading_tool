import subprocess
import numpy as np
import sys
import signal
import time

TAG = 'wE9ryARX'

def parse_transforms(data_string):
    try:
        transforms_string, buttons_string = data_string.split('&')
    except ValueError:
        return None, None

    transforms = {}
    split_transform_strings = transforms_string.split('|')

    for pair_string in split_transform_strings:
        transform = np.empty((4, 4))
        pair = pair_string.split(':')
        if len(pair) != 2:
            continue

        left_right_char = pair[0] 
        transform_string = pair[1]
        values = transform_string.split(' ')

        c = 0
        r = 0
        count = 0
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

def extract_pose_info(transform):
    x = transform[0, 3]
    y = transform[1, 3]
    z = transform[2, 3]
    
    R = transform[:3, :3]
    
    sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    if sy > 1e-6:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0

    return x, y, z, roll, pitch, yaw

def print_pose(hand_side, transform):
    
    x, y, z, roll, pitch, yaw = extract_pose_info(transform)

    side_name = "右手" if hand_side == 'r' else "左手"
    print(f"\n{'='*60}")
    print(f"{side_name} (Controller {hand_side.upper()})")
    print(f"{'='*60}")
    print(f"位置 (Position):")
    print(f"  X: {x:8.4f} m")
    print(f"  Y: {y:8.4f} m")
    print(f"  Z: {z:8.4f} m")
    print(f"\n姿态 (Orientation) [弧度]:")
    print(f"  Roll : {roll:8.4f} rad ({np.degrees(roll):8.2f}°)")
    print(f"  Pitch: {pitch:8.4f} rad ({np.degrees(pitch):8.2f}°)")
    print(f"  Yaw  : {yaw:8.4f} rad ({np.degrees(yaw):8.2f}°)")
    print(f"\n4x4 变换矩阵:")
    for i in range(4):
        print(f"  [{transform[i, 0]:8.4f}, {transform[i, 1]:8.4f}, {transform[i, 2]:8.4f}, {transform[i, 3]:8.4f}]")

def main():
    print("正在连接 Quest VR 设备...")

    result = subprocess.run(['adb', 'devices'], capture_output=True, text=True)
    if 'device' not in result.stdout:
        print("错误: 未检测到 Quest 设备，请确保:")
        print("1. Quest 已通过 USB 连接")
        print("2. 已开启开发者模式")
        print("3. 已允许 USB 调试")
        sys.exit(1)

    print("✓ Quest 设备已连接")
    print("\n正在启动 teleop 应用...")

    subprocess.run([
        'adb', 'shell', 'am', 'start',
        '-n', 'com.rail.oculus.teleop/com.rail.oculus.teleop.MainActivity',
        '-a', 'android.intent.action.MAIN',
        '-c', 'android.intent.category.LAUNCHER'
    ], capture_output=True)

    print("✓ teleop 应用已启动")
    print("\n开始读取手柄位姿数据...")
    print("按 Ctrl+C 停止\n")

    process = subprocess.Popen(
        ['adb', 'logcat', '-T', '0'],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        bufsize=1
    )

    def signal_handler(sig, frame):
        print("\n\n正在停止...")
        process.terminate()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    try:
        for line in process.stdout:
            line = line.strip()
            if TAG in line:
                try:
                    data = line.split(TAG + ': ')[1]
                    transforms, buttons = parse_transforms(data)

                    if transforms:
                        
                        print(f"\n时间戳: {time.strftime('%H:%M:%S', time.localtime())}")

                        if 'l' in transforms:
                            print_pose('l', transforms['l'])
                        if 'r' in transforms:
                            print_pose('r', transforms['r'])

                        print(f"\n按钮状态: {buttons}")
                        print("-" * 60)

                except Exception as e:
                    pass

    except KeyboardInterrupt:
        print("\n\n正在停止...")
        process.terminate()

    finally:
        process.terminate()
        print("已停止")

if __name__ == '__main__':
    main()
