#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class CropRowNavigator(Node):
    def __init__(self):
        super().__init__('crop_row_navigator')
        self.bridge = CvBridge()
        
        # --- Navigation State Machine ---
        self.autonomous_mode = True 
        self.emergency_stop = False
        self.current_state = "NAVIGATING" # States: NAVIGATING, EOL_DRIVE_OUT, EOL_TURN
        self.state_start_time = 0.0
        
        # Subscriptions & Publishers
        self.subscription = self.create_subscription(Image, '/vision/crop_mask', self.mask_callback, 1)
        self.mode_subscription = self.create_subscription(String, '/control/mode', self.mode_callback, 1)
        self.emergency_subscription = self.create_subscription(Bool, '/control/emergency_stop', self.emergency_callback, 1)
        self.cmd_subscription = self.create_subscription(Twist, '/cmd_vel_manual', self.manual_cmd_callback, 1)
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/navigation/status', 10)
        self.debug_pub = self.create_publisher(Image, '/vision/navigation_debug', 1)
        
        # Tuning parameters
        self.kp = 0.02
        self.forward_speed = 0.1
        self.status_timer = self.create_timer(1.0, self.publish_status)
        self.get_logger().info("Advanced Triangle Scan Navigator Node started!")

    def mode_callback(self, msg):
        command = msg.data.lower()
        if command in ["auto", "autonomous"] and not self.emergency_stop:
            self.autonomous_mode = True
            self.current_state = "NAVIGATING"
        elif command in ["manual", "stop"]:
            self.autonomous_mode = False
            self.cmd_pub.publish(Twist())

    def emergency_callback(self, msg):
        self.emergency_stop = msg.data
        if self.emergency_stop:
            self.autonomous_mode = False
            self.cmd_pub.publish(Twist())

    def manual_cmd_callback(self, msg):
        if not self.autonomous_mode and not self.emergency_stop:
            self.cmd_pub.publish(msg)

    # --- The Triangle Scan Algorithm ---
    
    def anchor_scan(self, mask):
        """Step 1: Scans top of the mask for the Anchor Point (A)."""
        H, W = mask.shape
        h = int(0.2 * H) # Height of the scan ROI
        
        start_x = int(0.2 * W)
        end_x = int(0.8 * W)
        
        threshold = h * 0.15 * 255  
        
        # FIX: Scan up to 4 levels down instead of 3 to account for 0.4 FPS lag
        for level in range(4):
            roi_top = level * h
            roi_bottom = (level + 1) * h
            
            roi = mask[roi_top:roi_bottom, start_x:end_x]
            col_sums = np.sum(roi, axis=0)
            
            max_col_idx = np.argmax(col_sums)
            if col_sums[max_col_idx] > threshold:
                anchor_point = (start_x + max_col_idx, roi_top)
                
                # FIX: Delay EOL until level 3 (the bottom 60-80% of the image)
                if level == 0 or level == 1:
                    return anchor_point, "NORMAL"
                elif level == 2:
                    return anchor_point, "APPROACHING_EOL"
                elif level == 3:
                    return anchor_point, "EOL"
                    
        # If we went through all 4 levels and found nothing, the row has vanished!
        return None, "EOL_NO_ROW"

    def line_scan(self, mask, anchor):
        """Step 2: Scans from Anchor to bottom edge to find the best central line."""
        H, W = mask.shape
        A_x, A_y = anchor
        max_sum = -1
        best_p_x = A_x 
        
        search_range = int(0.4 * W) 
        start_x = max(0, A_x - search_range)
        end_x = min(W, A_x + search_range)
        
        for p_x in range(start_x, end_x, 10): 
            line_mask = np.zeros_like(mask)
            cv2.line(line_mask, (A_x, A_y), (p_x, H), 255, 6) 
            
            intersection = cv2.bitwise_and(mask, line_mask)
            current_sum = np.sum(intersection)
            
            if current_sum > max_sum:
                max_sum = current_sum
                best_p_x = p_x
                
        return (best_p_x, H)

    # --- Main Navigation Loop ---

    def mask_callback(self, msg):
        if not self.autonomous_mode or self.emergency_stop:
            return
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            H, W = cv_image.shape
            debug_img = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
            current_time = time.time()
            twist = Twist()
            
            # --- STATE 1: Standard Visual Servoing ---
            if self.current_state == "NAVIGATING":
                anchor, eol_state = self.anchor_scan(cv_image)
                
                # FIX: Catch BOTH EOL and complete row disappearance
                if eol_state in ["EOL", "EOL_NO_ROW"]:
                    self.get_logger().info(f"End of Line Detected ({eol_state})! Triggering Exit Maneuver.")
                    self.current_state = "EOL_DRIVE_OUT"
                    self.state_start_time = current_time
                
                elif anchor is None:
                    # Failsafe in case it's completely lost but not EOL
                    self.cmd_pub.publish(Twist())
                    cv2.putText(debug_img, "NO ROW VISIBLE", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                else:
                    p_r = self.line_scan(cv_image, anchor)
                    center_x = W / 2.0
                    error = center_x - p_r[0]
                    
                    twist.linear.x = self.forward_speed
                    twist.angular.z = float(self.kp * error)
                    
                    twist.angular.z = max(-1.0, min(1.0, twist.angular.z))
                    self.cmd_pub.publish(twist)
                    
                    cv2.circle(debug_img, anchor, 8, (0, 255, 255), -1)
                    cv2.line(debug_img, anchor, p_r, (0, 255, 0), 4)
                    cv2.circle(debug_img, p_r, 8, (0, 0, 255), -1)
                    cv2.putText(debug_img, f"NAVIGATING | Err: {error:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # --- STATE 2: Drive into Headland ---
            elif self.current_state == "EOL_DRIVE_OUT":
                if current_time - self.state_start_time < 2.5: 
                    twist.linear.x = self.forward_speed
                    self.cmd_pub.publish(twist)
                    cv2.putText(debug_img, "EOL: DRIVING OUT", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 165, 0), 2)
                else:
                    self.current_state = "EOL_TURN"
                    self.state_start_time = current_time

            # --- STATE 3: U-Turn Maneuver ---
            elif self.current_state == "EOL_TURN":
                # Note: Because the speed is faster, you may need to reduce '3.5' seconds
                if current_time - self.state_start_time < 1.5: 
                    # FIX: Cranked angular velocity to 2.0 for higher turning power
                    twist.angular.z = 2.0 
                    self.cmd_pub.publish(twist)
                    cv2.putText(debug_img, "EOL: TURNING", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
                else:
                    self.current_state = "NAVIGATING"
                    self.get_logger().info("Turn complete. Entering next row.")

            # Publish Debug Image
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8'))
            
        except Exception as e:
            self.get_logger().error(f"Navigation error: {e}")

    def publish_status(self):
        status_msg = String()
        status_msg.data = f"Mode: {'Auto' if self.autonomous_mode else 'Manual'} | State: {self.current_state}"
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    navigator = CropRowNavigator()
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        navigator.cmd_pub.publish(Twist()) # Stop motors
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()