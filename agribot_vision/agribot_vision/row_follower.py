#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import qos_profile_sensor_data 

class CropRowFollower(Node):
    def __init__(self):
        super().__init__('crop_row_follower')
        
        self.bridge = CvBridge()
        
        # Subscriber for Pi Cam (Using Sensor Data QoS for performance)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw', 
            self.image_callback, 
            qos_profile_sensor_data
        )
            
        # IMPORTANT: Publish to /cmd_vel_raw so the Lidar Node can intercept/override if needed
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        
        # Control Parameters
        self.target_linear_speed = 0.2  # m/s (Reduced for safety during testing)
        self.kp = 0.005                 # Proportional gain for steering
        self.min_area = 2000            # Minimum pixel area to consider a "row"

        self.get_logger().info("Crop Row Follower (Inverted Mount Fix) Started.")

    def image_callback(self, msg):
        try:
            # 1. Convert ROS Image to OpenCV
            raw_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 2. FIX: Rotate 180 degrees because camera is mounted upside down
            cv_image = cv2.rotate(raw_frame, cv2.ROTATE_180)
            
        except Exception as e:
            self.get_logger().error(f"CV Bridge/Rotation Error: {e}")
            return

        height, width, _ = cv_image.shape
        
        # 3. ROI: Focus on the bottom half of the (now correctly oriented) image
        crop_img = cv_image[int(height/2):height, 0:width]

        # 4. Color Masking (Green Rows)
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        lower_green = np.array([35, 40, 40])
        upper_green = np.array([90, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # 5. Contour Detection
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        cmd = Twist()

        if contours:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            
            if area > self.min_area:
                M = cv2.moments(c)
                if M['m00'] > 0:
                    cx = int(M['m10']/M['m00'])
                    
                    # Center of image is width/2
                    # Error > 0 means row is to the RIGHT
                    # Error < 0 means row is to the LEFT
                    error = cx - (width / 2)
                    
                    cmd.linear.x = self.target_linear_speed
                    
                    # STEERING LOGIC: 
                    # If error is positive (row is right), we need a negative angular.z to turn right.
                    cmd.angular.z = float(-error * self.kp) 
                    
                    # Debugging log (throttle this in production)
                    # self.get_logger().info(f"Area: {area:.0f} | Error: {error} | Steer: {cmd.angular.z:.2f}")
            else:
                self.get_logger().info("Target area lost (too small). Stopping.", throttle_duration_sec=2.0)
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
        else:
            self.get_logger().info("No green rows detected. Stopping.", throttle_duration_sec=2.0)
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
                
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = CropRowFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()