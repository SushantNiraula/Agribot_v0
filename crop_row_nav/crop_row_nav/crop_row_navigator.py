#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from agribot_interfaces.action import CropInspection # Ensure this matches your package
import cv2
import numpy as np
import math
import time
from rclpy.qos import qos_profile_sensor_data

class LightweightCropNavigator(Node):
    def __init__(self):
        super().__init__('crop_row_navigator')
        self.bridge = CvBridge()

        # --- State Machine ---
        self.current_state = "NAVIGATING" # NAVIGATING, EOL_TURN, INSPECTING
        self.autonomous_mode = True
        self.has_locked_on_row = False

        # --- Data Buffers ---
        self.latest_frame = None
        self.latest_mask = None
        self.current_yaw = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.last_frame_update = time.time()
        
        # --- Arm Inspection Trackers ---
        self.last_inspect_x = None
        self.last_inspect_y = None

        # --- Configurable Parameters ---
        self.forward_speed = 0.1       # m/s
        self.kp = 0.005                # Steering sensitivity
        self.inspection_interval = 1.5 # Meters between arm inspections
        # EOL Debounce: Require ~3 seconds of pure dirt before turning
        self.eol_counter = 0
        self.eol_frames_required = 30 
        
        # --- CALIBRATED VISION PARAMETERS ---
        self.num_strips = 4
        self.lower_green = np.array([39, 63, 21])
        self.upper_green = np.array([119, 255, 255])

        # --- Subscriptions ---
        self.sub_camera = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, qos_profile_sensor_data)        
        self.sub_odom = self.create_subscription(Odometry, '/wheel/odometry', self.odom_callback, 10)        
        self.sub_mode = self.create_subscription(String, '/control/mode', self.mode_callback, 1)

        # --- Publishers & Actions ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.debug_pub = self.create_publisher(Image, '/vision/navigation_debug', 1)
        self.arm_client = ActionClient(self, CropInspection, '/arm/inspect_crop')

        # --- 10Hz Timer ---
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Lightweight CV System Online. Waiting for Camera Feed...")

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        
        if self.last_inspect_x is None:
            self.last_inspect_x = self.current_x
            self.last_inspect_y = self.current_y

    def camera_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Resize frame to save CPU cycles on Raspberry Pi
        frame = cv2.resize(frame, (320, 240))
        
        # --- FLIP THE IMAGE 180 DEGREES ---
        self.latest_frame = cv2.rotate(frame, cv2.ROTATE_180)
        
        self.last_frame_update = time.time()

    def mode_callback(self, msg):
        if msg.data.lower() in ["manual", "stop"]:
            self.autonomous_mode = False
            self.stop_robot()
            self.get_logger().info("Switched to MANUAL mode.")
        else:
            self.autonomous_mode = True
            self.get_logger().info("Switched to AUTO mode.")

    # --- LIGHTWEIGHT COMPUTER VISION PIPELINE ---
    def process_image(self, frame):
        height, width, _ = frame.shape
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Color Segmentation
        mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        self.latest_mask = mask # Save for debug publisher
        
        # Horizontal Strips logic
        roi_top = height // 2
        strip_height = (height - roi_top) // self.num_strips
        centroids = []
        
        for i in range(self.num_strips):
            y_start = roi_top + (i * strip_height)
            y_end = y_start + strip_height
            strip = mask[y_start:y_end, 0:width]
            
            contours, _ = cv2.findContours(strip, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                # Ignore tiny noise blobs
                if cv2.contourArea(largest_contour) > 50: 
                    M = cv2.moments(largest_contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = y_start + int(M["m01"] / M["m00"])
                        centroids.append((cx, cy))

        # Linear Regression to find anchor
        if len(centroids) >= 2:
            x_coords = np.array([pt[0] for pt in centroids])
            y_coords = np.array([pt[1] for pt in centroids])
            
            fit = np.polyfit(y_coords, x_coords, 1)
            poly = np.poly1d(fit)
            
            # Predict where the row hits the bottom of the screen
            target_x_bottom = int(poly(height))
            target_x_bottom = max(0, min(width, target_x_bottom))
            
            return (target_x_bottom, height), "FOUND", centroids, poly
            
        elif len(centroids) == 1:
            return (centroids[0][0], centroids[0][1]), "FOUND", centroids, None
            
        return None, "EOL", [], None

    # --- ARM ACTION LOGIC ---
    def trigger_inspection(self):
        self.get_logger().info(f"Traveled {self.inspection_interval}m. Stopping for inspection...")
        self.current_state = "INSPECTING"
        self.stop_robot() 
        
        if not self.arm_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("Arm Action Server not available! Skipping.")
            self.resume_navigation()
            return
            
        goal_msg = CropInspection.Goal()
        goal_msg.start = True
        
        self.send_goal_future = self.arm_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.resume_navigation()
            return
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.resume_navigation()

    def resume_navigation(self):
        self.last_inspect_x = self.current_x
        self.last_inspect_y = self.current_y
        self.current_state = "NAVIGATING"

    # --- MAIN CONTROL LOOP ---
    def control_loop(self):
        if not self.autonomous_mode or self.latest_frame is None or self.last_inspect_x is None:
            return

        if (time.time() - self.last_frame_update) > 2.0:
            self.stop_robot()
            self.get_logger().warn("Camera Feed Delayed! Standing by...", throttle_duration_sec=5.0)
            return

        if self.current_state == "INSPECTING":
            return

        if self.current_state == "NAVIGATING":
            dist_for_inspect = math.hypot(self.current_x - self.last_inspect_x, self.current_y - self.last_inspect_y)
            if dist_for_inspect >= self.inspection_interval:
                self.trigger_inspection()
                return

        twist = Twist()
        anchor, status, centroids, poly = self.process_image(self.latest_frame)
        H, W = self.latest_frame.shape[:2]

        if self.current_state == "NAVIGATING":
            if status == "FOUND":
                self.has_locked_on_row = True
                self.eol_counter = 0 
                
                # Calculate steering error
                error = (W / 2) - anchor[0]
                twist.linear.x = self.forward_speed
                twist.angular.z = float(self.kp * error)
                
            elif status == "EOL" and self.has_locked_on_row:
                self.eol_counter += 1
                if self.eol_counter >= self.eol_frames_required:
                    self.get_logger().info("Row ended! Spinning to find the next one...")
                    self.current_state = "EOL_TURN"  # Jump straight to turning
                    self.eol_counter = 0
                else:
                    # Coast forward slightly while deciding if it's really the end
                    twist.linear.x = self.forward_speed 
            else:
                # Lost the row but wasn't locked on yet, move slowly
                twist.linear.x = self.forward_speed * 0.5 

        elif self.current_state == "EOL_TURN":
            # Are we looking at a row again?
            if status == "FOUND":
                self.get_logger().info("Row found! Resuming navigation.")
                self.current_state = "NAVIGATING"
                self.last_inspect_x = self.current_x
                self.last_inspect_y = self.current_y
                self.has_locked_on_row = True
            else:
                # Spin in place at high speed until the camera sees green crops
                twist.linear.x = 0.0   # 0.0 means stop moving forward
                
                # Turn speed (Increase this if you want it to whip around faster)
                # Use a POSITIVE number to spin Left. Use NEGATIVE to spin Right.
                fast_turn_speed = -1.2 
                twist.angular.z = fast_turn_speed

        self.cmd_pub.publish(twist)
        self.publish_debug(anchor, centroids, poly)

    def publish_debug(self, anchor, centroids, poly):
        if self.latest_frame is None or self.latest_mask is None: return

        debug_img = self.latest_frame.copy()
        H, W = debug_img.shape[:2]
        
        # Apply red tint to detected crop pixels
        red_overlay = np.zeros_like(debug_img)
        red_overlay[:] = (0, 0, 255) 
        mask_bool = self.latest_mask > 0
        blended = cv2.addWeighted(debug_img, 0.6, red_overlay, 0.4, 0)
        debug_img[mask_bool] = blended[mask_bool]

        # Draw the state
        status_color = (0, 255, 0) if self.current_state == "NAVIGATING" else (0, 165, 255)
        if self.current_state == "INSPECTING": status_color = (0, 255, 255)
        cv2.putText(debug_img, f"STATE: {self.current_state}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
        
        # Draw the detected centroids
        for pt in centroids:
            cv2.circle(debug_img, pt, 5, (0, 255, 255), -1)
            
        # Draw the regression line
        if poly is not None:
            roi_top = H // 2
            top_x = int(poly(roi_top))
            bottom_x = int(poly(H))
            cv2.line(debug_img, (top_x, roi_top), (bottom_x, H), (255, 0, 0), 2)

        # Draw the final anchor and steering line
        if anchor:
            cv2.circle(debug_img, anchor, 8, (255, 255, 255), -1) 
            cv2.line(debug_img, (W//2, H), anchor, (0, 255, 0), 2)

        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8'))

def main():
    rclpy.init()
    node = LightweightCropNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()