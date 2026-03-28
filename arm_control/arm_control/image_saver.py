#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2
import os
import datetime

class ImageSaverNode(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.bridge = CvBridge()
        self.latest_image = None
        
        # Create a directory in your home folder to store the images
        self.save_dir = os.path.expanduser('~/agribot_crop_images')
        os.makedirs(self.save_dir, exist_ok=True)
        
        # Listen to the camera
        self.sub = self.create_subscription(Image, '/esp32cam/image_raw', self.image_callback, 10)
        
        # Create the service that the Arm Controller will call
        self.srv = self.create_service(Trigger, '/save_crop_image', self.save_image_callback)
        
        self.get_logger().info(f"📸 Image Saver Ready! Images will be saved to: {self.save_dir}")

    def image_callback(self, msg):
        # Always keep the most recent frame in memory
        self.latest_image = msg

    def save_image_callback(self, request, response):
        if self.latest_image is None:
            response.success = False
            response.message = "Cannot save: No image received from camera yet!"
            self.get_logger().warn(response.message)
            return response
            
        try:
            # Convert ROS Image to OpenCV Image
            cv_img = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")
            
            # Generate a unique filename using the current date and time
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(self.save_dir, f"crop_data_{timestamp}.jpg")
            
            # Save the image
            cv2.imwrite(filename, cv_img)
            
            response.success = True
            response.message = f"Successfully saved: {filename}"
            self.get_logger().info(response.message)
            
        except Exception as e:
            response.success = False
            response.message = f"Failed to save image: {str(e)}"
            self.get_logger().error(response.message)
            
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ImageSaverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()