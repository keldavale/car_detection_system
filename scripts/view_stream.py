#!/usr/bin/env python3

import cv2
import socket
import pickle
import struct
import argparse

def main():
    parser = argparse.ArgumentParser(description='View car detection video stream')
    parser.add_argument('--host', type=str, default='localhost',
                      help='IP address of the Raspberry Pi')
    parser.add_argument('--port', type=int, default=8089,
                      help='Port number for streaming')
    args = parser.parse_args()

    # Connect to server
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print(f"Connecting to {args.host}:{args.port}...")
    client_socket.connect((args.host, args.port))
    print("Connected!")

    data = b""
    payload_size = struct.calcsize(">L")

    try:
        while True:
            # Get message size
            while len(data) < payload_size:
                data += client_socket.recv(4096)
            packed_msg_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack(">L", packed_msg_size)[0]

            # Get frame data
            while len(data) < msg_size:
                data += client_socket.recv(4096)
            frame_data = data[:msg_size]
            data = data[msg_size:]

            # Decode and display frame
            frame = pickle.loads(frame_data)
            frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
            cv2.imshow('Car Detection View', frame)

            if cv2.waitKey(1) == ord('q'):
                break

    except KeyboardInterrupt:
        print("\nExiting...")
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        client_socket.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main() 