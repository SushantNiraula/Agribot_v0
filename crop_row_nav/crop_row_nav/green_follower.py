#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class GreenFollower(Node):
    def __init__(self):
        super().__init__('green_follower')
        self.bridge = CvBridge()
        
        # Subscribe to the 5 FPS throttled image to save CPU
        self.image_sub = self.create_subscription(
            Image, 
            '/vision/image_throttled/compressed',  # Updated topic
            self.image_callback, 
            10
        )
        
        # Publisher for motor control
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Publisher for debugging what the robot "sees"
        self.debug_pub = self.create_publisher(Image, '/vision/green_debug', 1)
        
        self.kp = 0.005          
        self.forward_speed = 0.1 # Explicitly set to 0.1 m/s
        self.min_blob_size = 500
        
        self.get_logger().info("🟢 Green Follower Node Started! Hunting for plants...")

    def image_callback(self, msg):
        try:
            # 1. Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width, _ = cv_image.shape
            
            # 2. Convert BGR to HSV (Hue, Saturation, Value) for better color tracking
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # 3. Define the color range for GREEN in HSV
            # You might need to tweak these depending on your lab lighting/camera
            lower_green = np.array([35, 50, 50])
            upper_green = np.array([85, 255, 255])
            
            # 4. Create a binary mask (white where green is, black everywhere else)
            mask = cv2.inRange(hsv, lower_green, upper_green)
            
            # 5. Calculate the center of mass of the green pixels
            M = cv2.moments(mask)
            twist = Twist()
            
            if M["m00"] > self.min_blob_size:
                # Calculate the X and Y coordinates of the center
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                # Draw a target on the debug image
                cv2.circle(cv_image, (cx, cy), 10, (0, 0, 255), -1)
                cv2.putText(cv_image, "TARGET LOCKED", (cx - 50, cy - 20), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                
                # Calculate steering error (Image Center X - Target Center X)
                center_x = width / 2
                error = center_x - cx
                
                # Apply Proportional Control
                twist.linear.x = self.forward_speed
                twist.angular.z = float(self.kp * error)
                
                # Cap the turn speed for safety
                twist.angular.z = max(-1.0, min(1.0, twist.angular.z))
                
            else:
                # No green found (or too small), stop the robot
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                cv2.putText(cv_image, "NO GREEN DETECTED", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # Publish the motor commands
            self.cmd_pub.publish(twist)
            
            # Publish the debug image so you can view it in rqt_image_view
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8'))
            
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GreenFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot before shutting down
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()