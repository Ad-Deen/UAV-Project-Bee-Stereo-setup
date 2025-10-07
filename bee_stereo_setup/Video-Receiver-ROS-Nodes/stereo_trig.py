import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class CameraTriggerPublisher(Node):
    def __init__(self):
        super().__init__('camera_trigger_pub')
        self.publisher_ = self.create_publisher(Bool, '/camera/trigger', 10)
        self.timer = self.create_timer(0.05, self.publish_trigger)  # 10 Hz trigger

    def publish_trigger(self):
        msg = Bool()
        msg.data = True
        self.publisher_.publish(msg)
        # self.get_logger().info('Published trigger: True')

def main(args=None):
    rclpy.init(args=args)
    node = CameraTriggerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
