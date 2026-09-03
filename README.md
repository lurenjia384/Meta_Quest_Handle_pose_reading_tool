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
