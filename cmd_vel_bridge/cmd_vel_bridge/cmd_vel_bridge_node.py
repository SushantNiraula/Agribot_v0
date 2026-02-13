import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__("cmd_vel_bridge")

        self.declare_parameter("in_topic", "/cmd_vel")
        self.declare_parameter("out_topic", "/diff_drive_controller/cmd_vel")
        self.declare_parameter("frame_id", "base_link")

        in_topic = self.get_parameter("in_topic").value
        out_topic = self.get_parameter("out_topic").value
        self.frame_id = self.get_parameter("frame_id").value

        self.pub = self.create_publisher(TwistStamped, out_topic, 10)
        self.sub = self.create_subscription(Twist, in_topic, self.cb, 10)

        self.get_logger().info(f"Bridging {in_topic} (Twist) -> {out_topic} (TwistStamped)")

    def cb(self, msg: Twist):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.twist = msg
        self.pub.publish(out)


def main():
    rclpy.init()
    node = CmdVelBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
