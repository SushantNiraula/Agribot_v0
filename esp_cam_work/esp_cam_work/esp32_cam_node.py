#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import urllib.request

class Esp32CamBridge(Node):
    def __init__(self):
        super().__init__('esp32_cam_bridge')
        
        # --- Parameter: Change this to your ESP32's actual IP ---
        self.declare_parameter('esp32_ip', '192.168.1.100') 
        esp_ip = self.get_parameter('esp32_ip').get_parameter_value().string_value
        self.stream_url = f"http://{esp_ip}/stream"
        
        self.get_logger().info(f"Connecting to ESP32-CAM stream at: {self.stream_url}")
        
        # --- Publishers ---
        self.image_pub = self.create_publisher(Image, '/esp32cam/image_raw', 10)
        self.compressed_pub = self.create_publisher(CompressedImage, '/esp32cam/image_raw/compressed', 10)
        
        self.bridge = CvBridge()
        self.byte_buffer = bytes()
        self.stream = None
        
        # Run timer to process the stream
        self.timer = self.create_timer(0.05, self.publish_frame)

    def publish_frame(self):
        # 1. Reconnect if the Wi-Fi connection is lost or hasn't started
        if self.stream is None:
            try:
                self.stream = urllib.request.urlopen(self.stream_url, timeout=3)
            except Exception as e:
                self.get_logger().warn(f"Waiting for ESP32-CAM... ({e})", throttle_duration_sec=2.0)
                return

        # 2. Read raw chunks of the MJPEG stream
        try:
            self.byte_buffer += self.stream.read(4096)
            
            # Look for the start (0xFF 0xD8) and end (0xFF 0xD9) markers of a JPEG image
            a = self.byte_buffer.find(b'\xff\xd8')
            b = self.byte_buffer.find(b'\xff\xd9')
            
            if a != -1 and b != -1:
                # Extract the exact JPEG bytes
                jpg_bytes = self.byte_buffer[a:b+2]
                
                # Clear the processed bytes from the buffer
                self.byte_buffer = self.byte_buffer[b+2:]
                
                # CORRECT:
                msg_header = Header()
                msg_header.stamp = self.get_clock().now().to_msg()
                msg_header.frame_id = "esp32_camera_link"
                
                # --- Publish Compressed Image DIRECTLY (Saves Massive CPU!) ---
                compressed_msg = CompressedImage()
                compressed_msg.header = msg_header
                compressed_msg.format = "jpeg"
                compressed_msg.data = jpg_bytes # Push raw bytes directly, no cv2 encode needed!
                self.compressed_pub.publish(compressed_msg)
                
                # --- Publish Raw Image (Decoded specifically for local Pi viewing) ---
                frame = cv2.imdecode(np.frombuffer(jpg_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is not None:
                    img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                    img_msg.header = msg_header
                    self.image_pub.publish(img_msg)
                    
        except Exception as e:
            self.get_logger().error(f"Stream interrupted: {e}")
            self.stream = None # Forces a reconnect on the next timer loop

def main(args=None):
    rclpy.init(args=args)
    node = Esp32CamBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # Fix for the rclpy shutdown crash: only shut down if it hasn't been already
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()