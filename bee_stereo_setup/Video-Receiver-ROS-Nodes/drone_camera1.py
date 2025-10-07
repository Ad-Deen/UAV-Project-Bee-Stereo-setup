#!/usr/bin/env python3

import socket
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os 


# Saving params
# SAVE_DIR = "bee/perception/stereo_left"
# os.makedirs(SAVE_DIR, exist_ok=True)

UDP_IP = "192.168.4.2"
# UDP_IP = "192.168.0.113"
UDP_PORT = 12345    #cam1

MAX_UDP_PACKET_SIZE = 1400 # Maximum size of a single UDP packet 1024
HEADER_SIZE = 6  # frame_id (2), chunk_no (2), chunk_size (2)

frame_buffers = {}
frame_chunks_count = {}

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
print(f"Listening on UDP {UDP_IP}:{UDP_PORT}")


class UdpCamNode(Node):
    def __init__(self):
        super().__init__('udp_cam_node')
        self.publisher_ = self.create_publisher(Image, '/drone_cam1', 10)
        self.bridge = CvBridge()
        self.last_frame_id = -1
        self.timer = self.create_timer(0.001, self.receive_udp)
        # Counter for saving frames
        self.frame_counter = 1  
    def process_frame(self, data):
        np_arr = np.frombuffer(data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img is not None:
            rotated_img = cv2.flip(img, -1)
            # Publish to ROS2
            msg = self.bridge.cv2_to_imgmsg(rotated_img, encoding='bgr8')
            self.publisher_.publish(msg)

            # Display image
            cv2.imshow('ESP32 Stream 1', rotated_img)
            cv2.waitKey(1)
            # Handle key press
            # key = cv2.waitKey(1) & 0xFF
            # if key == ord('s'):  # save on "s"
            #     filename = os.path.join(SAVE_DIR, f"{self.frame_counter}.png")
            #     cv2.imwrite(filename, rotated_img)
            #     self.get_logger().info(f"Saved frame as {filename}")
            #     self.frame_counter += 1
        else:
            self.get_logger().warn("Failed to decode image")

    def process_frame_with_padding(self, frame_id):
        chunks_dict = frame_buffers[frame_id]
        max_chunk_no = max(chunks_dict.keys())
        expected_chunks = max_chunk_no + 1

        full_frame = b''
        for i in range(expected_chunks):
            if i in chunks_dict:
                full_frame += chunks_dict[i]
            else:
                full_frame += b'\x00' * MAX_UDP_PACKET_SIZE

        self.get_logger().info(f"[Frame {frame_id}] Showing with padding, size: {len(full_frame)} bytes")
        self.process_frame(full_frame)

    def receive_udp(self):
        try:
            packet, addr = sock.recvfrom(MAX_UDP_PACKET_SIZE + HEADER_SIZE)
        except BlockingIOError:
            return

        if len(packet) < HEADER_SIZE:
            return

        frame_id = (packet[0] << 8) | packet[1]
        chunk_no = (packet[2] << 8) | packet[3]
        chunk_size = (packet[4] << 8) | packet[5]
        data = packet[6:6+chunk_size]

        if frame_id not in frame_buffers:
            frame_buffers[frame_id] = {}

        frame_buffers[frame_id][chunk_no] = data

        if frame_id != self.last_frame_id and self.last_frame_id in frame_buffers:
            self.process_frame_with_padding(self.last_frame_id)
            del frame_buffers[self.last_frame_id]

        if self.last_frame_id == -1 or frame_id != self.last_frame_id:
            self.last_frame_id = frame_id


def main(args=None):
    rclpy.init(args=args)
    node = UdpCamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
