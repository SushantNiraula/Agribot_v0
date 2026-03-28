#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import time

class CropRowNavigator(Node):
    def __init__(self):
        super().__init__('crop_row_navigator')
        self.bridge = CvBridge()

        # --- State Machine ---
        self.current_state = "NAVIGATING" # NAVIGATING, EOL_DRIVE_OUT, EOL_TURN
        self.autonomous_mode = True
        self.emergency_stop = False
        self.has_locked_on_row = False

        # --- Data Buffers ---
        self.latest_mask = None
        self.latest_ai_frame = None
        self.current_yaw = 0.0
        self.start_x = 0.0
        self.start_y = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.start_yaw = 0.0
        self.last_ai_update = time.time()

        # --- Configurable Parameters ---
        self.forward_speed = 0.1   # m/s
        self.kp = 0.02             # Steering sensitivity
        self.drive_out_dist = 0.3  # Meters to clear row before turning
        self.turn_speed = 0.7      # Rad/s for U-turn

        # --- Subscriptions ---
        self.sub_ai_frame = self.create_subscription(Image, '/vision/ai_input_debug', self.ai_frame_callback, 1)
        self.sub_mask = self.create_subscription(Image, '/vision/crop_mask', self.mask_callback, 1)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_mode = self.create_subscription(String, '/control/mode', self.mode_callback, 1)

        # --- Publishers ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.debug_pub = self.create_publisher(Image, '/vision/navigation_debug', 1)

        # --- 10Hz Timer (The "Brain" that keeps moving even if AI is slow) ---
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Final Crop Navigator Online. Waiting for AI Mask from Laptop...")

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def ai_frame_callback(self, msg):
        self.latest_ai_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def mask_callback(self, msg):
        self.latest_mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        self.last_ai_update = time.time() # Reset the timeout clock

    def mode_callback(self, msg):
        if msg.data.lower() in ["manual", "stop"]:
            self.autonomous_mode = False
            self.cmd_pub.publish(Twist())
            self.get_logger().info("Switched to MANUAL mode.")
        else:
            self.autonomous_mode = True
            self.get_logger().info("Switched to AUTO mode.")

    def scan_for_anchor(self, mask):
        """Reference Paper Algorithm: Scans bottom-up for the row start"""
        H, W = mask.shape
        # Check 4 horizontal slices
        for level in [0.8, 0.6, 0.4, 0.2]:
            y_coord = int(H * level)
            row_slice = mask[y_coord, :]
            
            # FIX: Require at least 10 white pixels (255 * 10 = 2550) to ignore tiny noise
            if np.sum(row_slice) > 2550: 
                x_coord = int(np.mean(np.where(row_slice > 0)))
                return (x_coord, y_coord), "FOUND"
        return None, "EOL"

    def control_loop(self):
        # 1. If not in auto mode, or waiting for the VERY first mask, do nothing
        if not self.autonomous_mode or self.latest_mask is None:
            return

        # 2. Relaxed Safety: Stop if AI hasn't updated in 10 seconds (Laptop lag/WiFi spike)
        if (time.time() - self.last_ai_update) > 10.0:
            self.cmd_pub.publish(Twist())
            # Use modulo math to only print this warning once every 5 seconds so it doesn't spam
            if int(time.time()) % 5 == 0:
                self.get_logger().warn("AI Feed Delayed! Standing by...")
            return

        twist = Twist()
        anchor, status = self.scan_for_anchor(self.latest_mask)
        H, W = self.latest_mask.shape

        # --- State 1: Follow the Row ---
        if self.current_state == "NAVIGATING":
            if status == "FOUND":
                self.has_locked_on_row = True
                error = (W / 2) - anchor[0]
                twist.linear.x = self.forward_speed
                # Steer towards the center of the crop row
                twist.angular.z = float(self.kp * error)
            elif status == "EOL" and self.has_locked_on_row:
                self.get_logger().info("Row end detected. Driving out...")
                self.current_state = "EOL_DRIVE_OUT"
                self.start_x, self.start_y = self.current_x, self.current_y
            else:
                twist.linear.x = self.forward_speed * 0.5 # Creep forward to find row

        # --- State 2: Drive out of the row ---
        elif self.current_state == "EOL_DRIVE_OUT":
            dist = math.hypot(self.current_x - self.start_x, self.current_y - self.start_y)
            if dist < self.drive_out_dist:
                twist.linear.x = self.forward_speed
            else:
                self.get_logger().info("Drive out complete. Starting 180 Turn.")
                self.current_state = "EOL_TURN"
                self.start_yaw = self.current_yaw

        # --- State 3: 180 Degree U-Turn ---
        elif self.current_state == "EOL_TURN":
            diff = abs(math.atan2(math.sin(self.current_yaw - self.start_yaw), 
                                 math.cos(self.current_yaw - self.start_yaw)))
            if diff < 3.0: # Roughly 172 degrees to prevent overshooting
                twist.angular.z = self.turn_speed
            else:
                self.get_logger().info("Turn finished. Navigating back.")
                self.current_state = "NAVIGATING"
                self.has_locked_on_row = False

        self.cmd_pub.publish(twist)
        self.publish_debug(anchor)

    def publish_debug(self, anchor):
        if self.latest_ai_frame is None or self.latest_mask is None:
            return

        # Sync mask to AI frame size
        m_h, m_w = self.latest_mask.shape
        f_h, f_w = self.latest_ai_frame.shape[:2]
        mask_vis = cv2.resize(self.latest_mask, (f_w, f_h))

        # Build color overlay
        debug_img = self.latest_ai_frame.copy()
        green_overlay = np.zeros_like(debug_img)
        green_overlay[:] = (0, 255, 0)
        
        # Apply mask as 40% transparency
        mask_bool = mask_vis > 0
        debug_img[mask_bool] = cv2.addWeighted(debug_img, 0.6, green_overlay, 0.4, 0)[mask_bool]

        # Draw UI
        status_color = (0, 255, 0) if self.current_state == "NAVIGATING" else (0, 165, 255)
        cv2.putText(debug_img, f"STATE: {self.current_state}", (10, 30), 2, 0.7, status_color, 2)
        
        if anchor:
            # Scale anchor for visualization if mask was resized
            v_anchor = (int(anchor[0] * (f_w/m_w)), int(anchor[1] * (f_h/m_h)))
            cv2.circle(debug_img, v_anchor, 10, (0, 0, 255), -1)
            cv2.line(debug_img, (f_w//2, f_h), v_anchor, (255, 0, 0), 3)

        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8'))

def main():
    rclpy.init()
    node = CropRowNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()