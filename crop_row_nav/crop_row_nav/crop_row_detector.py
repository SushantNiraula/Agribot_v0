#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# Updated import for LiteRT on Python 3.12
from ai_edge_litert.interpreter import Interpreter

class CropRowDetector(Node):
    def __init__(self):
        super().__init__('crop_row_detector')
        
        # Hardcoded path to your model
        model_path = '/home/agribot/project_ws/src/Agribot_v0/crop_row_nav/models/unet_quantized.tflite'     
        self.get_logger().info("Loading LiteRT Model with 4 Threads...")
        
        # OPTIMIZATION 1: Force LiteRT to use all 4 cores of the Raspberry Pi
        self.interpreter = Interpreter(model_path=model_path, num_threads=4)
        self.interpreter.allocate_tensors()
        
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        
        self.bridge = CvBridge()
        
        # OPTIMIZATION 2: Add a processing lock to drop frames and prevent memory crashing
        self.is_processing = False
        
        # Subscribe to your robot's camera
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw', 
            self.image_callback,
            1) # Reduced queue size from 10 to 1 to drop stale frames instantly
            
        self.publisher_ = self.create_publisher(Image, '/vision/crop_mask', 1)
        
        self.get_logger().info("Crop Row Detector Node is running!")

    def image_callback(self, msg):
        # If the AI is still processing the last frame, completely ignore this new one
        if self.is_processing:
            return
            
        self.is_processing = True # Lock the processor
        
        try:
            # 1. Convert ROS Image to OpenCV Image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # FIX 3: Rotate the image 180 degrees to fix the physical upside-down mount
            cv_image = cv2.rotate(cv_image, cv2.ROTATE_180)
            
            # 2. Preprocess
            img_resized = cv2.resize(cv_image, (512, 512))
            img_normalized = img_resized.astype(np.float32) / 255.0
            input_data = np.expand_dims(img_normalized, axis=0)
            
            # 3. Feed to AI
            self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
            self.interpreter.invoke()
            
            # 4. Get output
            output_data = self.interpreter.get_tensor(self.output_details[0]['index'])
            mask = output_data[0] 
            
            # 5. Convert to Black & White image
            mask = np.where(mask > 0.5, 255, 0).astype(np.uint8)
            
            # 6. Publish
            mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
            self.publisher_.publish(mask_msg)
            
        finally:
            # Unlock the processor so it can grab the newest frame
            self.is_processing = False

def main(args=None):
    rclpy.init(args=args)
    detector = CropRowDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()