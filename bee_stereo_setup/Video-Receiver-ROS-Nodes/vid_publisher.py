#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


VIDEO_PATH = "/home/deen/ros2_ws/src/bee/bee/perception/stereo_output.avi"  # <-- change to your stacked video file


class StereoVideoPublisher(Node):
    def __init__(self):
        super().__init__("stereo_video_publisher")
        self.bridge = CvBridge()

        # Publishers for left and right topics
        self.pub_left = self.create_publisher(Image, "/drone_cam1", 10)
        self.pub_right = self.create_publisher(Image, "/drone_cam2", 10)

        # OpenCV video capture
        self.cap = cv2.VideoCapture(VIDEO_PATH)
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open video {VIDEO_PATH}")
            raise RuntimeError("Video not found or cannot be opened.")

        # Get video FPS for timing
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        if fps <= 0:
            fps = 5.0
        self.timer_period = 1.0 / fps

        # Start timer callback
        self.timer = self.create_timer(self.timer_period, self.publish_frames)

    def publish_frames(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info("End of video reached. Restarting...")
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            return

        # Split frame into left and right
        h, w, _ = frame.shape
        mid = w // 2
        left_frame = frame[:, :mid, :]
        right_frame = frame[:, mid:, :]

        # Convert to ROS Image messages
        msg_left = self.bridge.cv2_to_imgmsg(left_frame, encoding="bgr8")
        msg_right = self.bridge.cv2_to_imgmsg(right_frame, encoding="bgr8")

        # Publish
        self.pub_left.publish(msg_left)
        self.pub_right.publish(msg_right)

        # (Optional) Show for debugging
        cv2.imshow("Left Frame (/drone_cam1)", left_frame)
        cv2.imshow("Right Frame (/drone_cam2)", right_frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = StereoVideoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
