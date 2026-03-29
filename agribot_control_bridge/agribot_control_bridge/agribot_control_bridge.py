import socketio
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import CompressedImage
import base64
import time


class ControlBridge(Node):
    def __init__(self):
        super().__init__("control_bridge")

        # 🔧 Parameters
        self.declare_parameter("flask_url", "http://10.219.135.117:5000")
        self.declare_parameter("robot_id", "agribot-01")

        self.flask_url = self.get_parameter("flask_url").value
        self.robot_id = self.get_parameter("robot_id").value

        # 🔄 State
        self.inspection_active = False
        self.last_frame_time = 0.0
        self.frame_interval = 0.3   # send every 300ms (≈3 FPS)

        # 📡 Publishers
        self.mode_pub = self.create_publisher(String, "/control/mode", 10)
        self.emergency_pub = self.create_publisher(Bool, "/control/emergency_stop", 10)

        # 📥 Subscribers
        self.create_subscription(
            String,
            '/arm/inspect_crop/_action/feedback',
            self.feedback_callback,
            10
        )

        self.create_subscription(
            CompressedImage,
            '/esp32/arm_action',
            self.camera_callback,
            10
        )

        # 🔌 Socket.IO
        self.sio = socketio.Client(reconnection=True)

        self.sio.on("connect", self.on_connect)
        self.sio.on("control_mode", self.on_mode)
        self.sio.on("emergency_stop", self.on_emergency)

        # 🆕 Listen inspection lifecycle
        self.sio.on("inspection_started", self.on_inspection_start)
        self.sio.on("inspection_stopped", self.on_inspection_stop)

        self.sio.connect(self.flask_url)

        self.get_logger().info("ROS ↔ Socket bridge started")

    # =========================================
    # 🔌 SOCKET EVENTS
    # =========================================
    def on_connect(self):
        self.get_logger().info("Connected to Flask")

    def on_mode(self, data):
        if data.get("robot_id") != self.robot_id:
            return

        msg = String()
        msg.data = data.get("mode", "manual")
        self.mode_pub.publish(msg)

    def on_emergency(self, data):
        if data.get("robot_id") != self.robot_id:
            return

        msg = Bool()
        msg.data = bool(data.get("stop", False))
        self.emergency_pub.publish(msg)

    def on_inspection_start(self, data):
        if data.get("robot_id") != self.robot_id:
            return

        self.inspection_active = True
        self.get_logger().info("Inspection STARTED")

    def on_inspection_stop(self, data):
        if data.get("robot_id") != self.robot_id:
            return

        self.inspection_active = False
        self.get_logger().info("Inspection STOPPED")

    # =========================================
    # 📢 FEEDBACK CALLBACK
    # =========================================
    def feedback_callback(self, msg):
        try:
            text = msg.data.lower()

            # 🔥 Convert to structured event
            event = "general"
            status = None

            if "dry" in text:
                event = "soil"
                status = "dry"
            elif "wet" in text:
                event = "soil"
                status = "wet"

            payload = {
                "robot_id": self.robot_id,
                "event": event,
                "status": status,
                "message": msg.data,
                "timestamp": time.time()
            }

            self.sio.emit("arm_feedback", payload)
            self.get_logger().info("arm_feedback sent")

        except Exception as e:
            self.get_logger().error(f"Feedback emit error: {e}")

    # =========================================
    # 📷 CAMERA CALLBACK
    # =========================================
    def camera_callback(self, msg):
        try:
            now = time.time()

            # 🔥 Throttle FPS
            if now - self.last_frame_time < self.frame_interval:
                return
            self.last_frame_time = now

            # 🔥 Only send when inspection active (IMPORTANT)
            if not self.inspection_active:
                return

            # Convert image → base64
            image_base64 = base64.b64encode(msg.data).decode("utf-8")

            payload = {
                "robot_id": self.robot_id,
                "image_base64": image_base64,
                "timestamp": now
            }

            self.sio.emit("arm_camera", payload)
            self.get_logger().info("camera callback frame sent")

        except Exception as e:
            self.get_logger().error(f"Camera emit error: {e}")


def main():
    rclpy.init()
    node = ControlBridge()
    rclpy.spin(node)
    rclpy.shutdown()