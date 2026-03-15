#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

# ADDED: Import the QoS profile here
from rclpy.qos import qos_profile_sensor_data 

class CropRowFollower(Node):
    def __init__(self):
        super().__init__('crop_row_follower')
        
        self.bridge = CvBridge()
        
        # Subscribe using the Sensor Data QoS profile
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw', 
            self.image_callback, 
            qos_profile_sensor_data
        )
            
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Crop Row Follower Node Started. Waiting for images...")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return

        height, width, _ = cv_image.shape
        crop_img = cv_image[int(height/2):height, 0:width]

        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        
        lower_green = np.array([35, 50, 50])
        upper_green = np.array([85, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        # Default state: STOP
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0

        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            
            # FIX 1: Only move if the green object is large enough (> 1000 pixels)
            if area > 2000:
                M = cv2.moments(c)
                if M['m00'] > 0:
                    cx = int(M['m10']/M['m00'])
                    error = cx - (width / 2)
                    
                    cmd.linear.x = 0.3
                    
                    # FIX 2: Flipped the sign on the error to reverse steering direction
                    cmd.angular.z = float(error * 0.01) 
            else:
                self.get_logger().info("Green object too small. Stopping.")
        else:
            self.get_logger().info("No green detected. Stopping.")
                
        self.cmd_vel_pub.publish(cmd)
def main(args=None):
    rclpy.init(args=args)
    node = CropRowFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()