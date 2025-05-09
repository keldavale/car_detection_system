#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import Bool
import depthai as dai
import numpy as np
import cv2
from cv_bridge import CvBridge
import json
from gpiozero import PWMOutputDevice, AngularServo
import math
from collections import deque
from datetime import datetime
import blobconverter
import time
import socket
import pickle
import struct

class CarDetectionNode(Node):
    def __init__(self):
        super().__init__('car_detection_node')
        
        # Display parameters
        self.declare_parameter('display_mode', 'local')  # 'local', 'network', or 'none'
        self.declare_parameter('stream_port', 5000)
        self.declare_parameter('stream_host', '0.0.0.0')
        
        self.display_mode = self.get_parameter('display_mode').value
        self.stream_port = self.get_parameter('stream_port').value
        self.stream_host = self.get_parameter('stream_host').value
        
        # Initialize network streaming if needed
        self.server_socket = None
        self.client_socket = None
        if self.display_mode == 'network':
            self.setup_network_streaming()
        
        # Debug parameters
        self.declare_parameter('debug_mode', True)
        self.declare_parameter('show_fps', True)
        self.declare_parameter('save_debug_video', False)
        self.debug_mode = self.get_parameter('debug_mode').value
        self.show_fps = self.get_parameter('show_fps').value
        self.save_debug_video = self.get_parameter('save_debug_video').value
        
        # Debug variables
        self.frame_count = 0
        self.fps_start_time = time.time()
        self.fps = 0
        if self.save_debug_video:
            self.video_writer = None
        
        # Initialize GPIO for vibration motor and servo
        self.MOTOR_PIN = 18
        self.SERVO_PIN = 12
        self.vibration_pwm = PWMOutputDevice(self.MOTOR_PIN, frequency=100)
        self.servo = AngularServo(self.SERVO_PIN, min_angle=-90, max_angle=90)
        self.servo.angle = 0  # Center position
        
        # Parameters
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('proximity_threshold', 1.0)
        self.declare_parameter('min_angle', -30.0)
        self.declare_parameter('max_angle', 30.0)
        self.declare_parameter('min_velocity', 5.0)
        self.declare_parameter('danger_zone_width', 0.4)
        self.declare_parameter('max_vibration_distance', 2.0)
        self.declare_parameter('min_vibration_distance', 0.3)
        
        # Get parameters
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.proximity_threshold = self.get_parameter('proximity_threshold').value
        self.min_angle = math.radians(self.get_parameter('min_angle').value)
        self.max_angle = math.radians(self.get_parameter('max_angle').value)
        self.min_velocity = self.get_parameter('min_velocity').value
        self.danger_zone_width = self.get_parameter('danger_zone_width').value
        self.max_vibration_distance = self.get_parameter('max_vibration_distance').value
        self.min_vibration_distance = self.get_parameter('min_vibration_distance').value
        
        # Initialize DepthAI pipeline
        self.pipeline = self.create_pipeline()
        self.device = dai.Device(self.pipeline)
        
        # Get output queues
        self.rgb_queue = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        self.detection_queue = self.device.getOutputQueue(name="detections", maxSize=4, blocking=False)
        self.depth_queue = self.device.getOutputQueue(name="depth", maxSize=4, blocking=False)
        
        # ROS publishers and subscribers
        self.cv_bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, '/oak_d/rgb/image_raw', 10)
        self.detection_pub = self.create_publisher(Bool, '/car_detected', 10)
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/scan',
            self.lidar_callback,
            10)
        
        # Initialize tracking
        self.prev_frame = None
        self.prev_detections = {}
        self.detection_history = {}
        self.history_length = 5
        
        # Create timer for processing camera data
        self.timer = self.create_timer(0.033, self.process_camera)
        
        self.get_logger().info('Car Detection Node initialized')

    def create_pipeline(self):
        """Create DepthAI pipeline with RGB, stereo cameras, and object detection"""
        pipeline = dai.Pipeline()

        # 1. Define sources
        camRgb = pipeline.createColorCamera()
        left = pipeline.createMonoCamera()
        right = pipeline.createMonoCamera()
        stereo = pipeline.createStereoDepth()
        
        # 2. Create neural network
        spatialDetectionNetwork = pipeline.createMobileNetSpatialDetectionNetwork()
        
        # 3. Create outputs
        xoutRgb = pipeline.createXLinkOut()
        xoutDepth = pipeline.createXLinkOut()
        xoutNN = pipeline.createXLinkOut()

        xoutRgb.setStreamName("rgb")
        xoutDepth.setStreamName("depth")
        xoutNN.setStreamName("detections")

        # 4. Properties
        # RGB
        camRgb.setPreviewSize(512, 512)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        camRgb.setFps(30)

        # Mono cameras
        left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        right.setBoardSocket(dai.CameraBoardSocket.CAM_C)

        # Stereo
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
        stereo.setLeftRightCheck(True)
        stereo.setExtendedDisparity(False)
        stereo.setSubpixel(True)
        stereo.setDepthAlign(dai.CameraBoardSocket.RGB)  # Align depth to RGB frame
        stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)  # Updated median filter setting

        # Neural network
        # Using blobconverter to get the model
        model_path = blobconverter.from_zoo(
            name="vehicle-detection-0202",
            shaves=3,
            version="2021.4"
        )
        spatialDetectionNetwork.setBlobPath(model_path)
        spatialDetectionNetwork.setConfidenceThreshold(0.3)  # Lower threshold to catch more detections
        spatialDetectionNetwork.input.setBlocking(False)
        spatialDetectionNetwork.setBoundingBoxScaleFactor(0.8)  # Increase bounding box size
        spatialDetectionNetwork.setDepthLowerThreshold(100)
        spatialDetectionNetwork.setDepthUpperThreshold(5000)

        # 5. Linking
        # Link mono cameras to stereo
        left.out.link(stereo.left)
        right.out.link(stereo.right)

        # Link RGB camera to neural network
        camRgb.preview.link(spatialDetectionNetwork.input)
        
        # Link stereo depth to neural network
        stereo.depth.link(spatialDetectionNetwork.inputDepth)

        # Link outputs
        spatialDetectionNetwork.out.link(xoutNN.input)
        spatialDetectionNetwork.passthrough.link(xoutRgb.input)
        camRgb.preview.link(xoutRgb.input)
        stereo.depth.link(xoutDepth.input)

        return pipeline

    def calculate_velocity(self, detection_id, current_pos, timestamp):
        """Calculate velocity of a detected car based on its position history."""
        if detection_id not in self.detection_history:
            self.detection_history[detection_id] = deque(maxlen=self.history_length)
        
        self.detection_history[detection_id].append((current_pos, timestamp))
        
        if len(self.detection_history[detection_id]) < 2:
            return 0.0
        
        # Calculate velocity using the oldest and newest positions
        old_pos, old_time = self.detection_history[detection_id][0]
        new_pos, new_time = self.detection_history[detection_id][-1]
        
        time_diff = (new_time - old_time).total_seconds()
        if time_diff == 0:
            return 0.0
        
        # Calculate displacement in pixels
        displacement = np.linalg.norm(np.array(new_pos) - np.array(old_pos))
        
        # Convert pixel displacement to meters (approximate conversion)
        # Assuming the camera's field of view and resolution
        pixels_per_meter = 50  # This value should be calibrated for your setup
        displacement_meters = displacement / pixels_per_meter
        
        return displacement_meters / time_diff

    def calculate_safe_direction(self, frame_width, moving_cars):
        """
        Calculate the safest direction based on moving car positions.
        Returns angle in degrees (0-180) for servo and a safety score (0-1).
        """
        if not moving_cars:
            return 90, 1.0  # No cars detected, center position is safe
        
        # Create a danger map across the frame width
        danger_map = np.zeros(frame_width)
        
        for car in moving_cars:
            center_x, velocity = car['center_x'], car['velocity']
            
            # Calculate danger zone width based on velocity
            danger_width = int(frame_width * self.danger_zone_width * min(velocity / 10.0, 1.0))
            
            # Create gaussian danger distribution around car position
            x = np.arange(frame_width)
            danger = np.exp(-((x - center_x) ** 2) / (2 * (danger_width/2) ** 2))
            danger_map += danger
        
        # Normalize danger map
        if np.max(danger_map) > 0:
            danger_map /= np.max(danger_map)
        
        # Find the safest point (minimum in danger map)
        safest_x = np.argmin(danger_map)
        safety_score = 1.0 - danger_map[safest_x]
        
        # Convert to servo angle (0-180)
        # Invert the angle to point away from danger
        frame_center = frame_width / 2
        if safest_x < frame_center:
            # Danger on right, point left
            servo_angle = 0
        else:
            # Danger on left, point right
            servo_angle = 180
            
        return servo_angle, safety_score

    def set_servo_angle(self, angle_degrees):
        """Set servo angle in degrees"""
        self.servo.angle = angle_degrees - 90  # Convert to -90 to 90 range

    def set_vibration_intensity(self, intensity):
        """Set vibration motor intensity (0-100)"""
        self.vibration_pwm.value = intensity / 100.0

    def calculate_vibration_intensity(self, distance):
        """Calculate vibration intensity based on distance"""
        if distance >= self.max_vibration_distance:
            return 0
        elif distance <= self.min_vibration_distance:
            return 100
        
        # Linear mapping from distance to intensity
        # As distance decreases, intensity increases
        range_distance = self.max_vibration_distance - self.min_vibration_distance
        range_intensity = 100 - 0  # 0% to 100%
        
        intensity = ((self.max_vibration_distance - distance) / range_distance) * range_intensity
        return min(max(intensity, 0), 100)

    def create_debug_frame(self, frame, detections, depth_frame=None):
        """Create a debug frame with visualizations"""
        debug_frame = frame.copy()
        
        # Draw FPS if enabled
        if self.show_fps:
            self.frame_count += 1
            if time.time() - self.fps_start_time > 1:
                self.fps = self.frame_count / (time.time() - self.fps_start_time)
                self.frame_count = 0
                self.fps_start_time = time.time()
            cv2.putText(debug_frame, f"FPS: {self.fps:.1f}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Draw detections
        for detection in detections:
            # Denormalize bounding box
            bbox = self.frame_norm(frame, (detection.xmin, detection.ymin, detection.xmax, detection.ymax))
            
            # Get detection info
            conf = detection.confidence
            
            # Draw bounding box with thick green lines
            color = (0, 255, 0)  # Green
            thickness = 4  # Thicker lines
            
            # Draw the bounding box
            cv2.rectangle(debug_frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, thickness)
            
            # Draw label
            label = f"Car {conf*100:.1f}%"
            cv2.putText(debug_frame, label, (bbox[0], bbox[1] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        return debug_frame

    def frame_norm(self, frame, bbox):
        """Normalize bounding box coordinates"""
        normVals = np.full(len(bbox), frame.shape[0])
        normVals[::2] = frame.shape[1]
        return (np.clip(np.array(bbox), 0, 1) * normVals).astype(int)

    def process_camera(self):
        in_rgb = self.rgb_queue.tryGet()
        in_det = self.detection_queue.tryGet()
        in_depth = self.depth_queue.tryGet()

        if in_rgb is not None and in_depth is not None:
            frame = in_rgb.getCvFrame()
            depth_frame = in_depth.getFrame()
            current_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            detections = []
            if in_det is not None:
                detections = in_det.detections
                self.get_logger().info(f"Number of detections: {len(detections)}")
                for i, det in enumerate(detections):
                    self.get_logger().info(f"Detection {i}: Confidence: {det.confidence:.2f}, Label: {det.label}")
            
            # Create debug frame
            debug_frame = self.create_debug_frame(frame, detections, depth_frame)
            
            # Display the debug frame
            cv2.imshow("Car Detection", debug_frame)
            cv2.waitKey(1)  # Ensure the window is updated
            
            # Break loop on 'q' press
            if cv2.waitKey(1) == ord('q'):
                self.cleanup_and_shutdown()
                return
            
            if self.prev_frame is not None:
                if in_det is not None:
                    detections = in_det.detections
                    current_time = datetime.now()
                    moving_cars = []
                    
                    for detection in detections:
                        if detection.confidence >= self.confidence_threshold:
                            # Calculate center point of detection
                            x1, y1 = detection.xmin, detection.ymin
                            x2, y2 = detection.xmax, detection.ymax
                            center_x = (x1 + x2) / 2
                            center_y = (y1 + y2) / 2
                            
                            # Get spatial coordinates (in mm)
                            spatial_coords = detection.spatialCoordinates
                            distance = spatial_coords.z / 1000.0  # Convert to meters
                            
                            # Calculate velocity
                            detection_id = f"{x1:.2f}_{y1:.2f}"
                            velocity = self.calculate_velocity(detection_id, (center_x, center_y), current_time)
                            
                            if velocity >= self.min_velocity:
                                moving_cars.append({
                                    'center_x': center_x,
                                    'velocity': velocity,
                                    'distance': distance
                                })
                                self.get_logger().debug(
                                    f'Car detected at {distance:.2f}m moving at {velocity:.2f} m/s'
                                )
                    
                    if moving_cars:
                        # Calculate safe direction based on all moving cars
                        safe_angle, safety_score = self.calculate_safe_direction(frame.shape[1], moving_cars)
                        
                        # Update servo position to point to safety
                        self.set_servo_angle(safe_angle)
                        
                        # Log information
                        direction = "left" if safe_angle < 90 else "right"
                        self.get_logger().debug(
                            f'Moving cars detected! Safe direction: {direction}, '
                            f'Safety score: {safety_score:.2f}'
                        )
                        
                        self.detection_pub.publish(Bool(data=True))
                    else:
                        self.detection_pub.publish(Bool(data=False))
                        # Return servo to center when no cars detected
                        self.set_servo_angle(90)
            
            self.prev_frame = current_frame

    def is_angle_in_range(self, x, y):
        """Check if a point's angle is within the valid detection range."""
        angle = math.atan2(y, x)
        # Normalize angle to [-π, π]
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
            
        return self.min_angle <= angle <= self.max_angle

    def lidar_callback(self, msg):
        # Convert PointCloud2 to numpy array
        points = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 4)
        
        # Filter points based on angle
        valid_points = []
        for point in points:
            x, y = point[0], point[1]
            if self.is_angle_in_range(x, y):
                valid_points.append(point)
        
        if not valid_points:
            # No valid points, turn off vibration
            self.set_vibration_intensity(0)
            return
            
        valid_points = np.array(valid_points)
        
        # Calculate distances
        distances = np.linalg.norm(valid_points[:, :3], axis=1)
        closest_dist = np.min(distances)
        
        # Calculate and set vibration intensity based on closest distance
        vibration_intensity = self.calculate_vibration_intensity(closest_dist)
        self.set_vibration_intensity(vibration_intensity)
        
        if vibration_intensity > 0:
            closest_idx = np.argmin(distances)
            closest_point = valid_points[closest_idx]
            angle = math.degrees(math.atan2(closest_point[1], closest_point[0]))
            self.get_logger().debug(
                f'Close object detected! Distance: {closest_dist:.2f}m, '
                f'Angle: {angle:.1f}°, '
                f'Vibration: {vibration_intensity:.1f}%'
            )

    def cleanup_and_shutdown(self):
        """Clean shutdown of node and resources"""
        self.get_logger().info("Shutting down...")
        if self.save_debug_video and self.video_writer is not None:
            self.video_writer.release()
        cv2.destroyAllWindows()
        self.vibration_pwm.close()
        self.servo.close()
        rclpy.shutdown()

    def __del__(self):
        self.cleanup_and_shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = CarDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup_and_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 