#!/usr/bin/env python3

import depthai as dai
import cv2
import time
import numpy as np

def create_pipeline():
    """Create DepthAI pipeline with RGB and depth streams"""
    pipeline = dai.Pipeline()

    # Create RGB camera
    cam_rgb = pipeline.createColorCamera()
    cam_rgb.setPreviewSize(640, 480)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam_rgb.setInterleaved(False)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    cam_rgb.setFps(30)

    # Create mono cameras for depth
    left = pipeline.createMonoCamera()
    right = pipeline.createMonoCamera()
    left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    left.setBoardSocket(dai.CameraBoardSocket.LEFT)
    right.setBoardSocket(dai.CameraBoardSocket.RIGHT)

    # Create stereo depth
    stereo = pipeline.createStereoDepth()
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setLeftRightCheck(True)
    stereo.setExtendedDisparity(True)
    stereo.setSubpixel(True)

    # Create outputs
    xout_rgb = pipeline.createXLinkOut()
    xout_depth = pipeline.createXLinkOut()
    xout_rgb.setStreamName("rgb")
    xout_depth.setStreamName("depth")

    # Link nodes
    cam_rgb.preview.link(xout_rgb.input)
    left.out.link(stereo.left)
    right.out.link(stereo.right)
    stereo.depth.link(xout_depth.input)

    return pipeline

def main():
    print("Starting camera test...")
    
    # Create pipeline
    pipeline = create_pipeline()
    
    # Connect to device and start pipeline
    with dai.Device(pipeline) as device:
        print("Camera connected!")
        print(f"Device name: {device.getDeviceName()}")
        print(f"Device MXID: {device.getMxId()}")
        
        # Get output queues
        rgb_queue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        depth_queue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
        
        # Initialize FPS counter
        frame_count = 0
        fps_start_time = time.time()
        fps = 0
        
        while True:
            # Get frames
            in_rgb = rgb_queue.tryGet()
            in_depth = depth_queue.tryGet()
            
            if in_rgb is not None and in_depth is not None:
                # Get frames
                frame = in_rgb.getCvFrame()
                depth_frame = in_depth.getFrame()
                
                # Normalize depth for visualization
                depth_vis = cv2.normalize(depth_frame, None, 0, 255, cv2.NORM_MINMAX)
                depth_vis = cv2.applyColorMap(depth_vis.astype(np.uint8), cv2.COLORMAP_JET)
                
                # Calculate FPS
                frame_count += 1
                if time.time() - fps_start_time > 1:
                    fps = frame_count
                    frame_count = 0
                    fps_start_time = time.time()
                
                # Add FPS to frame
                cv2.putText(frame, f"FPS: {fps}", (10, 30),
                          cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                # Show frames
                cv2.imshow("RGB", frame)
                cv2.imshow("Depth", depth_vis)
                
                # Break loop on 'q' press
                if cv2.waitKey(1) == ord('q'):
                    break
    
    print("Camera test completed!")
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main() 