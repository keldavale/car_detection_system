# Car Detection System

A ROS 2 based car detection system using OAK-D Lite camera and RPLidar A1 sensor. The system detects moving cars, provides proximity alerts through a variable-intensity vibration motor, and controls a servo motor to indicate safe directions.

## Features

- Car detection using DepthAI neural network (vehicle-detection-0202 model from OpenVINO)
- Real-time depth sensing and spatial awareness
- LiDAR-based proximity detection with configurable detection zones
- Variable-intensity haptic feedback based on object proximity
- Servo motor control for indicating safe directions
- Multiple display options for debugging and monitoring
- Velocity-based filtering to focus on moving vehicles
- Network streaming capability for remote monitoring

## Neural Network Model

This system uses the `vehicle-detection-0202` model from Intel's OpenVINO Open Model Zoo. This model:
- Detects vehicles in various conditions
- Supports 256x256 input resolution
- Based on MobileNetV2 + SSD
- Trained on internal datasets with vehicles in various weather/lighting conditions
- Provides good balance of accuracy and performance for edge devices

### Model Setup

The model will be automatically downloaded and converted to the correct format when the system first runs, using `blobconverter`. However, you can also manually download and convert it:

```bash
# Install blobconverter if not already installed
python3 -m pip install blobconverter

# Download and convert model (will be cached for future use)
python3 -c "
import blobconverter
model_path = blobconverter.from_zoo(
    name='vehicle-detection-0202',
    shaves=6,
    version='2021.4'
)
print(f'Model saved to: {model_path}')
"
```

### Model Performance

- Input size: 256x256
- GFLOPs: 1.991
- MParams: 1.991
- Source framework: PyTorch

Expected detection performance:
- Vehicle detection accuracy: ~90% mAP on standard benchmarks
- Suitable for real-time processing on OAK-D Lite
- Optimized for edge deployment

### Inference Settings

The model is configured with the following default parameters in our system:
- Confidence threshold: 0.5 (adjustable via ROS parameter)
- Input resolution: 256x256 (scaled from camera input)
- Maximum number of detections: 100
- IOU threshold: 0.5

## Hardware Requirements

- Raspberry Pi (3B+ or newer recommended)
- OAK-D Lite camera
- RPLidar A1 sensor
- Vibration motor (connected to GPIO 18)
- Servo motor (connected to GPIO 12)

## Software Requirements

- ROS 2 Jazzy
- Python 3.8+
- OpenCV
- DepthAI
- NumPy
- RPi.GPIO

## Complete Raspberry Pi Setup Guide

### 1. Initial Raspberry Pi Setup
```bash
# Update system packages
sudo apt update && sudo apt upgrade -y

# Install system dependencies
sudo apt install -y python3-pip python3-venv git cmake
sudo apt install -y python3-opencv libopencv-dev
sudo apt install -y udev
```

### 2. Set Up ROS 2 Jazzy
```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Jazzy
sudo apt update
sudo apt install -y ros-jazzy-ros-base
sudo apt install -y ros-dev-tools

# Add ROS 2 setup to .bashrc
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3. Install DepthAI and RPLidar Dependencies
```bash
# Install DepthAI and model conversion tools
python3 -m pip install depthai
python3 -m pip install blobconverter

# Verify model availability
python3 -c "
import blobconverter
try:
    model_path = blobconverter.from_zoo(
        name='vehicle-detection-0202',
        shaves=6,
        version='2021.4'
    )
    print(f'Successfully verified model: {model_path}')
except Exception as e:
    print(f'Error verifying model: {e}')
"

# Install RPLidar ROS 2 package
sudo apt install -y ros-jazzy-rplidar-ros

# Set up udev rules for devices
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", SYMLINK+="rplidar"' | sudo tee /etc/udev/rules.d/rplidar.rules

# Reload udev rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 4. Hardware Setup

1. **OAK-D Lite Camera**:
   - Connect to USB 3.0 port (blue port) for best performance
   - Verify connection:
   ```bash
   python3 -c "import depthai; print(depthai.Device.getAllAvailableDevices())"
   ```

2. **RPLidar A1**:
   - Connect to any USB port
   - Note the device name (usually `/dev/ttyUSB0` or `/dev/rplidar`)
   - Verify connection:
   ```bash
   ls -l /dev/rplidar  # Should show the device if udev rules are working
   ```

3. **GPIO Connections**:
   ```
   Vibration Motor:
   - GPIO 18 (Pin 12) -> Motor Positive
   - Ground (Pin 6) -> Motor Negative

   Servo Motor:
   - GPIO 12 (Pin 32) -> Signal (Orange/Yellow)
   - 5V (Pin 2 or 4) -> Power (Red)
   - Ground (Pin 34) -> Ground (Brown/Black)
   ```

### 5. Create ROS 2 Workspace
```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone required repositories
git clone https://github.com/yourusername/car_detection_system.git
git clone https://github.com/Slamtec/rplidar_ros.git  # RPLidar ROS driver

# Install Python dependencies
cd car_detection_system
python3 -m pip install -r requirements.txt

# Install additional ROS 2 Jazzy dependencies
sudo apt install -y ros-jazzy-cv-bridge

# Build workspace
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

# Add workspace to .bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### 6. Testing the Setup

1. **Test RPLidar**:
```bash
# Terminal 1
ros2 launch rplidar_ros rplidar.launch.py

# Terminal 2 (verify data)
ros2 topic echo /scan
```

2. **Test OAK-D Lite**:
```bash
# Simple test script
python3 -c "
import depthai as dai
pipeline = dai.Pipeline()
cam = pipeline.createColorCamera()
xout = pipeline.createXLinkOut()
xout.setStreamName('rgb')
cam.preview.link(xout.input)
with dai.Device(pipeline) as device:
    q = device.getOutputQueue('rgb')
    print('Camera working!')
"
```

3. **Test GPIO**:
```bash
# Test script for GPIO
python3 -c "
import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)
GPIO.setup(12, GPIO.OUT)
pwm = GPIO.PWM(18, 100)
servo = GPIO.PWM(12, 50)
pwm.start(50)
servo.start(7.5)
time.sleep(2)
pwm.stop()
servo.stop()
GPIO.cleanup()
print('GPIO test complete!')
"
```

## Usage

### Basic Launch
```bash
ros2 run car_detection_system detection_node
```

### Display Modes

The system supports three display modes:

1. **Local Display** (for direct monitor connection):
```bash
ros2 run car_detection_system detection_node --ros-args -p display_mode:=local
```

2. **Network Streaming** (for remote viewing):
```bash
# On Raspberry Pi
ros2 run car_detection_system detection_node --ros-args -p display_mode:=network -p stream_host:=0.0.0.0 -p stream_port:=8089

# On remote machine (requires Python and OpenCV)
python3 scripts/view_stream.py --host <raspberry_pi_ip> --port 8089
```

3. **Headless Mode** (no visual output):
```bash
ros2 run car_detection_system detection_node --ros-args -p display_mode:=none
```

### Debug Mode
```bash
ros2 run car_detection_system detection_node --ros-args -p debug_mode:=true -p show_fps:=true
```

To save debug video:
```bash
ros2 run car_detection_system detection_node --ros-args -p debug_mode:=true -p save_debug_video:=true
```

## Configuration Parameters

### Detection Parameters
- `confidence_threshold` (float, default: 0.5): Minimum confidence for car detection
- `proximity_threshold` (float, default: 1.0): Distance threshold for proximity detection
- `min_velocity` (float, default: 5.0): Minimum velocity for moving car detection

### LiDAR Parameters
- `min_angle` (float, default: -30.0): Minimum angle for LiDAR detection zone
- `max_angle` (float, default: 30.0): Maximum angle for LiDAR detection zone
- `danger_zone_width` (float, default: 0.4): Width of the danger zone for servo control

### Haptic Feedback Parameters
- `max_vibration_distance` (float, default: 2.0): Maximum distance for vibration feedback
- `min_vibration_distance` (float, default: 0.3): Minimum distance for vibration feedback

### Display Parameters
- `display_mode` (string, default: 'local'): Display mode ('local', 'network', or 'none')
- `stream_port` (int, default: 8089): Port for network streaming
- `stream_host` (string, default: '0.0.0.0'): Host address for network streaming
- `debug_mode` (bool, default: true): Enable debug visualizations
- `show_fps` (bool, default: true): Show FPS counter in debug view
- `save_debug_video` (bool, default: false): Save debug view to video file

## Debug Visualization

The debug view includes:
- Bounding boxes around detected cars with confidence scores
- Distance measurements to detected objects
- LiDAR detection zone visualization
- Depth map overlay
- FPS counter
- Distance thresholds indicators
- Servo direction indicator

## Topics

### Published Topics
- `/oak_d/rgb/image_raw` (sensor_msgs/Image): Raw RGB camera feed
- `/car_detected` (std_msgs/Bool): Car detection status

### Subscribed Topics
- `/scan` (sensor_msgs/PointCloud2): LiDAR scan data

## Troubleshooting

### Common Issues

1. **Permission Denied for USB Devices**:
```bash
# Add user to required groups
sudo usermod -aG dialout $USER
sudo usermod -aG gpio $USER
# Log out and back in for changes to take effect
```

2. **RPLidar Not Found**:
```bash
# Check USB device
ls -l /dev/ttyUSB*
# If needed, manually specify port
ros2 launch rplidar_ros rplidar.launch.py serial_port:=/dev/ttyUSB0
```

3. **OAK-D Lite Not Detected**:
```bash
# Check USB connection
lsusb | grep 03e7
# Replug into USB 3.0 port
# Check udev rules are loaded
sudo udevadm control --reload-rules && sudo udevadm trigger
```

4. **GPIO Permission Issues**:
```bash
# Add user to gpio group
sudo usermod -aG gpio $USER
# Install GPIO library
sudo apt install -y python3-rpi.gpio
```

5. **ROS 2 Jazzy Package Issues**:
```bash
# If you encounter package not found errors
sudo apt update
sudo apt install -y ros-jazzy-<package_name>

# Common packages you might need:
sudo apt install -y ros-jazzy-cv-bridge
sudo apt install -y ros-jazzy-image-transport
sudo apt install -y ros-jazzy-image-transport-plugins
```

### Network Streaming Issues
1. Check if the port is open:
```bash
sudo netstat -tulpn | grep 8089
```

2. Verify network connectivity:
```bash
ping <raspberry_pi_ip>
```

3. Check firewall settings:
```bash
sudo ufw status
# If needed, allow port:
sudo ufw allow 8089
```

### Performance Optimization
- Adjust `maxSize` in output queues if experiencing frame drops
- Modify JPEG quality in network streaming (default: 50)
- Reduce frame resolution for better network performance 