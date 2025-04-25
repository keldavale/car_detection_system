#!/usr/bin/env python3

import depthai as dai
import cv2
import os
import sys

def main():
    # Check if we have a display
    try:
        if 'DISPLAY' not in os.environ:
            print("No display environment variable set")
            print("Try running with: DISPLAY=:0 python3 test_camera.py")
            return
            
        # Test display connection
        test_window_name = "Test Window"
        cv2.namedWindow(test_window_name, cv2.WINDOW_NORMAL)
        cv2.destroyWindow(test_window_name)
        print("Display connection successful")
    except Exception as e:
        print(f"Error connecting to display: {e}")
        print("Make sure X11 forwarding is enabled and DISPLAY variable is set correctly")
        return
    
    # Create pipeline
    pipeline = dai.Pipeline()

    # Define source and output
    cam = pipeline.create(dai.node.ColorCamera)
    xout = pipeline.create(dai.node.XLinkOut)

    xout.setStreamName("rgb")

    # Properties
    cam.setPreviewSize(640, 480)
    cam.setInterleaved(False)
    cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)

    # Linking
    cam.preview.link(xout.input)

    # Connect to device and start pipeline
    try:
        with dai.Device(pipeline) as device:
            print("Connected to OAK-D Lite camera")
            print("Press 'q' to quit")

            # Output queue will be used to get the rgb frames from the output defined above
            q = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

            # Create window
            window_name = "OAK-D Lite Camera"
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            
            try:
                while True:
                    in_rgb = q.get()  # blocking call, will wait until a new data has arrived
                    if in_rgb is not None:
                        frame = in_rgb.getCvFrame()
                        
                        # Add frame info overlay
                        height, width = frame.shape[:2]
                        cv2.putText(frame, f"Frame size: {width}x{height}", (10, 30),
                                  cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                        
                        # Display the frame
                        cv2.imshow(window_name, frame)
                        
                        # Break the loop if 'q' is pressed
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            break
            except KeyboardInterrupt:
                print("\nStopping camera test...")
            finally:
                cv2.destroyAllWindows()
    except Exception as e:
        print(f"Error accessing camera: {e}")
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main() 