#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist  # This is the standard ROS 2 motor command message!
from cv_bridge import CvBridge
import cv2
import numpy as np

class CropRowNavigator(Node):
    def __init__(self):
        super().__init__('crop_row_navigator')
        
        self.bridge = CvBridge()
        
        # 1. Subscribe to the AI's mask
        self.subscription = self.create_subscription(
            Image,
            '/vision/crop_mask',
            self.mask_callback,
            1)
            
        # 2. Publish motor commands to the wheels
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 3. (Optional) Publish a debug image so you can see the math working!
        self.debug_pub = self.create_publisher(Image, '/vision/navigation_debug', 1)

        # Tuning parameters
        self.kp = 0.01  # Proportional Gain (How hard to steer). Adjust this later!
        self.forward_speed = 0.3  # Drive at 0.1 meters per second
        
        self.get_logger().info("Navigator Node started! Waiting for AI masks...")

    def mask_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV (mono8 means black and white)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            height, width = cv_image.shape
            
            # STEP 1: Define the Region of Interest (ROI)
            # We only care about the bottom 150 pixels (what is directly in front of the robot)
            roi_top = height - 150
            roi = cv_image[roi_top:height, 0:width]
            
            # STEP 2: Find all the white pixels in that bottom slice
            white_pixels = cv2.findNonZero(roi)
            
            twist = Twist()
            
            # Create a color image for debugging
            debug_img = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
            cv2.rectangle(debug_img, (0, roi_top), (width, height), (255, 0, 0), 2) # Draw blue box around ROI
            
            if white_pixels is not None:
                # STEP 3: Find the center X coordinate of all the white pixels
                target_x = int(np.mean(white_pixels[:, 0, 0]))
                center_x = int(width / 2.0)
                
                # STEP 4: Calculate the Error
                error = center_x - target_x
                
                # Draw the target and center lines on our debug image
                cv2.line(debug_img, (center_x, 0), (center_x, height), (0, 255, 0), 2) # Green line = Image Center
                cv2.circle(debug_img, (target_x, roi_top + 75), 10, (0, 0, 255), -1)   # Red dot = AI Target
                
                # STEP 5: The P-Controller (Steering Math)
                # If target is on the right (target_x > center_x), error is negative -> turn right
                twist.linear.x = self.forward_speed
                twist.angular.z = float(self.kp * error)
                
                self.get_logger().info(f"Target: {target_x} | Error: {error} | Steering: {twist.angular.z:.3f}")
                
            else:
                # If there are NO white pixels, stop the robot so it doesn't crash!
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().warn("No crop row visible! Stopping.")
            
            # Publish the motor command
            self.cmd_pub.publish(twist)
            
            # Publish the debug image
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
            self.debug_pub.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error calculating navigation: {e}")

def main(args=None):
    rclpy.init(args=args)
    navigator = CropRowNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()