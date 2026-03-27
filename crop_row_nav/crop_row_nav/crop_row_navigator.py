#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
import cv2
import numpy as np

class CropRowNavigator(Node):
    def __init__(self):
        super().__init__('crop_row_navigator')
        
        self.bridge = CvBridge()
        
        # Navigation state
        self.autonomous_mode = True  # Start in autonomous mode
        self.emergency_stop = False
        self.current_command = Twist()  # Store current manual command
        
        # 1. Subscribe to the AI's mask
        self.subscription = self.create_subscription(
            Image,
            '/vision/crop_mask',
            self.mask_callback,
            1)
        
        # 2. Subscribe to mode control commands
        self.mode_subscription = self.create_subscription(
            String,
            '/control/mode',
            self.mode_callback,
            1)
        
        # 3. Subscribe to emergency stop
        self.emergency_subscription = self.create_subscription(
            Bool,
            '/control/emergency_stop',
            self.emergency_callback,
            1)
        
        # 4. Subscribe to manual commands (from Flutter app)
        self.cmd_subscription = self.create_subscription(
            Twist,
            '/cmd_vel_manual',
            self.manual_cmd_callback,
            1)
        
        # 5. Publish motor commands to the wheels
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 6. Publish navigation status
        self.status_pub = self.create_publisher(String, '/navigation/status', 10)
        
        # 7. (Optional) Publish a debug image
        self.debug_pub = self.create_publisher(Image, '/vision/navigation_debug', 1)
        
        # Tuning parameters
        self.kp = 0.05  # Proportional Gain
        self.forward_speed = 0.4  # Drive at 0.3 meters per second
        
        # Timer for status updates (every 1 second)
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info("Navigator Node started! Waiting for commands...")
        self.get_logger().info("Mode: MANUAL (waiting for start command)")
    
    def mode_callback(self, msg):
        """Handle mode switching commands from Flutter"""
        command = msg.data.lower()
        
        if command == "auto" or command == "autonomous":
            if not self.emergency_stop:
                self.autonomous_mode = True
                self.get_logger().info("=== Switching to AUTONOMOUS mode ===")
                self.publish_status()
            else:
                self.get_logger().warn("Cannot switch to autonomous mode - Emergency stop active!")
                
        elif command == "manual":
            self.autonomous_mode = False
            # Stop robot when switching to manual
            stop_cmd = Twist()
            self.cmd_pub.publish(stop_cmd)
            self.get_logger().info("=== Switching to MANUAL mode ===")
            self.publish_status()
            
        elif command == "stop":
            self.autonomous_mode = False
            stop_cmd = Twist()
            self.cmd_pub.publish(stop_cmd)
            self.get_logger().info("Stop command received")
            
    def emergency_callback(self, msg):
        """Handle emergency stop"""
        self.emergency_stop = msg.data
        
        if self.emergency_stop:
            self.autonomous_mode = False
            stop_cmd = Twist()
            self.cmd_pub.publish(stop_cmd)
            self.get_logger().warn("!!! EMERGENCY STOP ACTIVATED !!!")
        else:
            self.get_logger().info("Emergency stop released")
            
        self.publish_status()
    
    def manual_cmd_callback(self, msg):
        """Store manual commands from Flutter"""
        self.current_command = msg
        if not self.autonomous_mode and not self.emergency_stop:
            self.cmd_pub.publish(msg)
    
    def mask_callback(self, msg):
        """Process crop mask and generate autonomous navigation commands"""
        # Only run autonomous navigation if enabled and not emergency stopped
        if not self.autonomous_mode or self.emergency_stop:
            return
        
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            height, width = cv_image.shape
            
            # Define the Region of Interest (bottom 150 pixels)
            roi_top = height - 150
            roi = cv_image[roi_top:height, 0:width]
            
            # Find all the white pixels in the ROI
            white_pixels = cv2.findNonZero(roi)
            
            twist = Twist()
            
            # Create debug image
            debug_img = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
            cv2.rectangle(debug_img, (0, roi_top), (width, height), (255, 0, 0), 2)
            
            if white_pixels is not None and len(white_pixels) > 100:  # Minimum pixels to navigate
                # Find the center X coordinate of all white pixels
                target_x = int(np.mean(white_pixels[:, 0, 0]))
                center_x = int(width / 2.0)
                
                # Calculate the error
                error = center_x - target_x
                
                # Draw debug information
                cv2.line(debug_img, (center_x, 0), (center_x, height), (0, 255, 0), 2)
                cv2.circle(debug_img, (target_x, roi_top + 75), 10, (0, 0, 255), -1)
                
                # P-Controller for steering
                twist.linear.x = self.forward_speed
                twist.angular.z = float(self.kp * error)
                
                # Limit angular velocity
                max_angular = 1.0
                twist.angular.z = max(-max_angular, min(max_angular, twist.angular.z))
                
                self.get_logger().debug(f"Auto: Target: {target_x} | Error: {error} | Steering: {twist.angular.z:.3f}")
                
                # Add text to debug image
                cv2.putText(debug_img, f"Auto Mode | Error: {error:.1f}", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
            else:
                # No crop row detected - stop and warn
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().warn("No crop row visible! Stopping.")
                
                cv2.putText(debug_img, "NO CROP ROW DETECTED", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # Publish motor command
            self.cmd_pub.publish(twist)
            
            # Publish debug image
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
            self.debug_pub.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error calculating navigation: {e}")
    
    def publish_status(self):
        """Publish current navigation status"""
        status = {
            "mode": "autonomous" if self.autonomous_mode else "manual",
            "emergency_stop": self.emergency_stop,
            "timestamp": self.get_clock().now().nanoseconds / 1e9
        }
        
        # Also include additional info
        if self.autonomous_mode:
            status.update({
                "forward_speed": self.forward_speed,
                "kp": self.kp
            })
        
        status_msg = String()
        status_msg.data = str(status)
        self.status_pub.publish(status_msg)
        
        # Log status change
        self.get_logger().info(f"Status: Mode={status['mode']}, Emergency={status['emergency_stop']}")

def main(args=None):
    rclpy.init(args=args)
    navigator = CropRowNavigator()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure robot stops on shutdown
        stop_cmd = Twist()
        navigator.cmd_pub.publish(stop_cmd)
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()