import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HelloNode(Node):
    def __init__(self):
        super().__init__("hello_node")
        self.pub = self.create_publisher(String, "/hello", 10)
        self.timer = self.create_timer(0.5, self.on_timer)  # 2 Hz
        self.count = 0

    def on_timer(self):
        msg = String()
        msg.data = f"hello {self.count}"
        self.pub.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")
        self.count += 1


def main():
    rclpy.init()
    node = HelloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()