#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
# /home/deen/ros2_ws/src/bee/bee/perception/stereo_left
SAVE_LEFT_DIR = "/home/deen/ros2_ws/src/bee/bee/perception/dist_left"
SAVE_RIGHT_DIR = "/home/deen/ros2_ws/src/bee/bee/perception/dist_right"
os.makedirs(SAVE_LEFT_DIR, exist_ok=True)
os.makedirs(SAVE_RIGHT_DIR, exist_ok=True)


class StereoSaver(Node):
    def __init__(self):
        super().__init__('stereo_saver')
        self.bridge = CvBridge()
        self.left_image = None
        self.right_image = None
        self.frame_counter = 1

        # Subscribers
        self.create_subscription(Image, '/drone_cam1', self.left_callback, 10)
        self.create_subscription(Image, '/drone_cam2', self.right_callback, 10)

        # Timer to display
        self.create_timer(0.03, self.display_images)  # ~30 FPS loop

    def left_callback(self, msg):
        self.left_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def right_callback(self, msg):
        self.right_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def display_images(self):
        if self.left_image is not None:
            cv2.imshow("Left Camera (/drone_cam1)", self.left_image)
        if self.right_image is not None:
            cv2.imshow("Right Camera (/drone_cam2)", self.right_image)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('s') and self.left_image is not None and self.right_image is not None:
            left_path = os.path.join(SAVE_LEFT_DIR, f"{self.frame_counter}.png")
            right_path = os.path.join(SAVE_RIGHT_DIR, f"{self.frame_counter}.png")
            cv2.imwrite(left_path, self.left_image)
            cv2.imwrite(right_path, self.right_image)
            self.get_logger().info(f"Saved stereo pair {self.frame_counter}:")
            self.get_logger().info(f"  Left -> {left_path}")
            self.get_logger().info(f"  Right -> {right_path}")
            self.frame_counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = StereoSaver()
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
