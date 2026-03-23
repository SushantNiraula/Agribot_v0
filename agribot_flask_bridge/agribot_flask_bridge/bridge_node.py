import time
import math
import socketio
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry

import base64
from sensor_msgs.msg import CompressedImage


def yaw_from_quat(qx, qy, qz, qw) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def _safe_min(values, range_min, range_max) -> float | None:
    # Filter invalid values: NaN, inf, 0, out of range
    best = None
    for v in values:
        if v is None:
            continue
        if isinstance(v, float) and (math.isnan(v) or math.isinf(v)):
            continue
        if v <= 0:
            continue
        if v < range_min or v > range_max:
            continue
        if best is None or v < best:
            best = v
    return best


class ImuToFlaskBridge(Node):
    def __init__(self):
        super().__init__("imu_to_flask_bridge")

        self.declare_parameter("flask_url", "http://10.43.187.117:5000")
        self.declare_parameter("imu_topic", "/imu/data_raw")
        self.declare_parameter("odom_topic", "/odometry/filtered")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("emit_rate_hz", 10.0)
        self.declare_parameter("robot_id", "agribot-01")

        self.flask_url = str(self.get_parameter("flask_url").value)
        self.imu_topic = str(self.get_parameter("imu_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.emit_rate = float(self.get_parameter("emit_rate_hz").value)
        self.robot_id = str(self.get_parameter("robot_id").value)

        self._last_odom = None   # dict
        self._last_lidar = None  # dict

        self._last_image_time = 0.0
        self.image_rate = 2.0  # 2 FPS (safe)

        self.sio = socketio.Client(
            reconnection=True,
            reconnection_attempts=0,
            reconnection_delay=1,
            reconnection_delay_max=5,
            logger=False,
            engineio_logger=False,
        )

        self.sio.on("connect", self._on_connect)
        self.sio.on("disconnect", self._on_disconnect)
        self._connect_socket()

        self.sub_imu = self.create_subscription(
            Imu, self.imu_topic, self._on_imu, qos_profile_sensor_data
        )
        self.sub_odom = self.create_subscription(
            Odometry, self.odom_topic, self._on_odom, qos_profile_sensor_data
        )
        self.sub_scan = self.create_subscription(
            LaserScan, self.scan_topic, self._on_scan, qos_profile_sensor_data
        )

        self.sub_image = self.create_subscription(
            CompressedImage,
            "/camera/image_raw/compressed",
            self._on_image,
            qos_profile_sensor_data
        )

        self._last_emit = 0.0

        self.get_logger().info(
            f"IMU: {self.imu_topic} | ODOM: {self.odom_topic} | SCAN: {self.scan_topic} "
            f"| sending to {self.flask_url} @ {self.emit_rate} Hz"
        )

    def _connect_socket(self):
        try:
            if not self.sio.connected:
                self.sio.connect(self.flask_url, transports=["websocket", "polling"])
                self.get_logger().info("Connected to Flask SocketIO server.")
        except Exception as e:
            self.get_logger().warn(f"Socket connect failed: {e}")

    def _on_connect(self):
        self.get_logger().info("SocketIO connected.")

    def _on_disconnect(self):
        self.get_logger().warn("SocketIO disconnected.")

    # ---------------- ODOM ----------------
    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        t = msg.twist.twist

        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)

        if int(time.time()) % 5 == 0:
            self.get_logger().info(
                f"ODOM rx x={p.x:.2f} y={p.y:.2f} yaw={yaw:.2f} vx={t.linear.x:.2f} wz={t.angular.z:.2f}"
            )

        self._last_odom = {
            "x": float(p.x),
            "y": float(p.y),
            "yaw": float(yaw),
            "twist": {
                "vx": float(t.linear.x),
                "vy": float(t.linear.y),
                "wz": float(t.angular.z),
            },
            "frame_id": str(msg.header.frame_id),
            "child_frame_id": str(msg.child_frame_id),
        }

    # ---------------- LIDAR /SCAN ----------------
    def _on_scan(self, msg: LaserScan):
        ranges = list(msg.ranges)
        n = len(ranges)
        if n == 0:
            return

        # Define angular windows (in radians)
        # front: -15°..+15°, left: +60°..+120°, right: -120°..-60°
        def idx_from_angle(a):
            # Convert angle (rad) -> index
            if msg.angle_increment == 0:
                return 0
            i = int(round((a - msg.angle_min) / msg.angle_increment))
            return max(0, min(n - 1, i))

        def slice_angles(a0, a1):
            i0 = idx_from_angle(a0)
            i1 = idx_from_angle(a1)
            if i0 <= i1:
                return ranges[i0:i1 + 1]
            else:
                # wrap
                return ranges[i0:] + ranges[:i1 + 1]

        deg = math.pi / 180.0
        front = slice_angles(-15 * deg, 15 * deg)
        left = slice_angles(60 * deg, 120 * deg)
        right = slice_angles(-120 * deg, -60 * deg)

        min_front = _safe_min(front, msg.range_min, msg.range_max)
        min_left = _safe_min(left, msg.range_min, msg.range_max)
        min_right = _safe_min(right, msg.range_min, msg.range_max)

        self._last_lidar = {
            "frame_id": str(msg.header.frame_id),
            "min_front": float(min_front) if min_front is not None else None,
            "min_left": float(min_left) if min_left is not None else None,
            "min_right": float(min_right) if min_right is not None else None,
            "range_min": float(msg.range_min),
            "range_max": float(msg.range_max),
            "count": int(n),
        }

    # ---------------- IMU (emit loop trigger) ----------------
    def _on_imu(self, msg: Imu):
        now = time.time()
        if now - self._last_emit < (1.0 / max(self.emit_rate, 1.0)):
            return
        self._last_emit = now

        telemetry = {
            "ts": float(now),
            "robot_id": str(self.robot_id),
            "imu": {
                "frame_id": str(msg.header.frame_id),
                "ang_vel": {
                    "x": float(msg.angular_velocity.x),
                    "y": float(msg.angular_velocity.y),
                    "z": float(msg.angular_velocity.z),
                },
                "lin_acc": {
                    "x": float(msg.linear_acceleration.x),
                    "y": float(msg.linear_acceleration.y),
                    "z": float(msg.linear_acceleration.z),
                },
                "orientation_valid": bool(msg.orientation_covariance[0] != -1.0),
            },
        }

        if self._last_odom is not None:
            telemetry["odom"] = {
                "x": self._last_odom["x"],
                "y": self._last_odom["y"],
                "yaw": self._last_odom["yaw"],
            }
            telemetry["twist"] = self._last_odom["twist"]
            telemetry["speed_mps"] = abs(float(self._last_odom["twist"]["vx"]))

        if self._last_lidar is not None:
            telemetry["lidar"] = self._last_lidar

        try:
            if self.sio.connected:
                # self.get_logger().info("[DEBUG] Emitting telemetry...")
                # self.get_logger().info(f"[DEBUG DATA] {telemetry}")
                self.sio.emit("telemetry", telemetry)
            else:
                self._connect_socket()
        except Exception as e:
            self.get_logger().warn(f"Emit failed: {e}")
    

    def _on_image(self, msg: CompressedImage):
        now = time.time()

        # Rate limit (VERY IMPORTANT)
        if now - self._last_image_time < (1.0 / self.image_rate):
            return
        self._last_image_time = now

        try:
            # Convert to base64
            image_base64 = base64.b64encode(msg.data).decode("utf-8")

            payload = {
                "robot_id": self.robot_id,
                "ts": now,
                "image": image_base64
            }

            if self.sio.connected:
                self.sio.emit("camera", payload)
                self.get_logger().info("[CAM] Frame sent")
            else:
                self._connect_socket()

        except Exception as e:
            self.get_logger().warn(f"Camera emit failed: {e}")


def main():
    rclpy.init()
    node = ImuToFlaskBridge()
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