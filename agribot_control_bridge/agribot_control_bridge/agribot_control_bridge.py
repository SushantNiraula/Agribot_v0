import socketio
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

class ControlBridge(Node):
    def __init__(self):
        super().__init__("control_bridge")

        self.declare_parameter("flask_url", "http://10.63.158.117:5000")
        self.declare_parameter("robot_id", "agribot-01")

        self.flask_url = self.get_parameter("flask_url").value
        self.robot_id = self.get_parameter("robot_id").value

        self.mode_pub = self.create_publisher(String, "/control/mode", 10)
        self.emergency_pub = self.create_publisher(Bool, "/control/emergency_stop", 10)

        self.sio = socketio.Client(reconnection=True)

        self.sio.on("connect", self.on_connect)
        self.sio.on("control_mode", self.on_mode)
        self.sio.on("emergency_stop", self.on_emergency)

        self.sio.connect(self.flask_url)

    def on_connect(self):
        self.get_logger().info("Connected to Flask")

    def on_mode(self, data):
        if data.get("robot_id") != self.robot_id:
            return

        msg = String()
        msg.data = data.get("mode", "manual")

        self.mode_pub.publish(msg)
        self.get_logger().info(f"Mode changed to {msg.data}")

    def on_emergency(self, data):
        if data.get("robot_id") != self.robot_id:
            return

        msg = Bool()
        msg.data = bool(data.get("stop", False))

        self.emergency_pub.publish(msg)
        self.get_logger().warn(f"Emergency: {msg.data}")


def main():
    rclpy.init()
    node = ControlBridge()
    rclpy.spin(node)
    rclpy.shutdown()