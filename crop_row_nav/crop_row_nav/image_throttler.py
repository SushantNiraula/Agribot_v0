#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import time

class ImageThrottler(Node):
    def __init__(self):
        super().__init__('image_throttler')
        self.bridge = CvBridge()
        
        # We only want to send 5 frames per second
        self.target_fps = 5.0
        self.last_pub_time = 0.0
        
        # 1. Subscribe to the local RAW image (this is fast and doesn't use Wi-Fi)
        self.sub = self.create_subscription(
            Image, 
            '/camera/image_raw', 
            self.image_callback, 
            1)
            
        # 2. Publish our own CUSTOM compressed topic for the laptop
        self.pub = self.create_publisher(
            CompressedImage, 
            '/vision/image_throttled/compressed', 
            1)
            
        self.get_logger().info("Image Throttler running: Dropping frames and squishing JPEGs to 30% quality...")

    def image_callback(self, msg):
        current_time = time.time()
        
        # If it hasn't been 0.2 seconds yet, throw the frame in the trash!
        if (current_time - self.last_pub_time) < (1.0 / self.target_fps):
            return 
            
        self.last_pub_time = current_time
        
        try:
            # Convert to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Manually compress the image to a tiny JPEG (30% quality)
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 30]
            success, encoded_image = cv2.imencode('.jpg', cv_image, encode_param)
            
            if success:
                # Package it into a ROS 2 CompressedImage message
                compressed_msg = CompressedImage()
                compressed_msg.header = msg.header
                compressed_msg.format = "jpeg"
                compressed_msg.data = encoded_image.tobytes()
                
                # Send it over Wi-Fi to the laptop!
                self.pub.publish(compressed_msg)
                
        except Exception as e:
            self.get_logger().error(f"Throttler Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageThrottler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()