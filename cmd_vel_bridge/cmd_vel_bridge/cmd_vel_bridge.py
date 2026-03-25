import socketio
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
# from dotenv import load_dotenv
# import os

# load_dotenv()  # ✅ loads .env file


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__("cmd_vel_bridge")

        # flask_url_env = os.getenv("FLASK_URL")

        # if not flask_url_env:
        #     self.get_logger().error("FLASK_URL not set in environment!")
        #     raise RuntimeError("Missing FLASK_URL")

        self.declare_parameter("flask_url", "http://10.63.158.117:5000")
        self.declare_parameter("robot_id", "agribot-01")
        self.declare_parameter("cmd_topic", "/cmd_vel")

        self.flask_url = str(self.get_parameter("flask_url").value)
        self.robot_id = str(self.get_parameter("robot_id").value)
        self.cmd_topic = str(self.get_parameter("cmd_topic").value)

        self.pub = self.create_publisher(Twist, self.cmd_topic, 10)

        self.sio = socketio.Client(reconnection=True)

        self.sio.on("connect", self._on_connect)
        self.sio.on("disconnect", self._on_disconnect)
        self.sio.on("cmd_vel", self._on_cmd_vel)

        self._connect()

        self.get_logger().info(f"Listening cmd_vel from {self.flask_url} -> publishing {self.cmd_topic}")

    def _connect(self):
        try:
            self.sio.connect(self.flask_url, transports=["websocket", "polling"])
        except Exception as e:
            self.get_logger().warn(f"Socket connect failed: {e}")

    def _on_connect(self):
        self.get_logger().info("Socket connected (cmd_vel).")

    def _on_disconnect(self):
        self.get_logger().warn("Socket disconnected (cmd_vel).")

    def _on_cmd_vel(self, data):
        try:
            if not isinstance(data, dict):
                return
            if str(data.get("robot_id")) != self.robot_id:
                return  # ignore other robots

            cmd = data.get("cmd_vel") or {}
            vx = float(cmd.get("vx", 0.0))
            vy = float(cmd.get("vy", 0.0))
            wz = float(cmd.get("wz", 0.0))

            msg = Twist()
            msg.linear.x = vx
            msg.linear.y = vy
            msg.angular.z = wz

            self.pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"cmd_vel publish failed: {e}")

def main():
    rclpy.init()
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.sio.disconnect()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()