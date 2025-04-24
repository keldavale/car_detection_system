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

- ROS 2 (Humble or newer)
- Python 3.8+
- OpenCV
- DepthAI
- NumPy
- RPi.GPIO

## Installation

1. Clone this repository into your ROS 2 workspace:
```bash
cd ~/ros2_ws/src
git clone https://github.com/ykeldavale/car_detection_system.git
```

2. Install dependencies:
```bash
pip3 install depthai opencv-python numpy blobconverter
sudo apt-get install ros-humble-cv-bridge
```

3. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select car_detection_system
```

4. Source the workspace:
```bash
source ~/ros2_ws/install/setup.bash
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

### Display Issues
1. For X11 forwarding, ensure xauth is installed:
```bash
sudo apt-get install xauth
```

2. Connect with X11 forwarding enabled:
```bash
ssh -X pi@<raspberry_pi_ip>
```

### Performance Optimization
- Adjust `maxSize` in output queues if experiencing frame drops
- Modify JPEG quality in network streaming (default: 50)
- Reduce frame resolution for better network performance 