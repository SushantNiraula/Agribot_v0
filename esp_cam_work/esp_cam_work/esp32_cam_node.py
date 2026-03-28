#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import urllib.request
import threading
import time

class Esp32CamBridge(Node):
    def __init__(self):
        super().__init__('esp32_cam_bridge')
        
        self.declare_parameter('esp32_ip', '10.63.158.232') 
        esp_ip = self.get_parameter('esp32_ip').get_parameter_value().string_value
        self.stream_url = f"http://{esp_ip}/stream"
        
        self.image_pub = self.create_publisher(Image, '/esp32cam/image_raw', 10)
        self.compressed_pub = self.create_publisher(CompressedImage, '/esp32cam/custom_compressed', 10)
        
        self.bridge = CvBridge()
        
        # Start a background thread to ingest the stream as fast as possible
        self.thread = threading.Thread(target=self.run_stream_reader, daemon=True)
        self.thread.start()
        self.get_logger().info(f"Stream reader thread started for: {self.stream_url}")

    def run_stream_reader(self):
        """Continuous loop running in a separate thread to saturate bandwidth."""
        byte_buffer = bytes()
        stream = None

        while rclpy.ok():
            if stream is None:
                try:
                    req = urllib.request.Request(self.stream_url, headers={'User-Agent': 'Mozilla/5.0'})
                    stream = urllib.request.urlopen(req, timeout=5)
                    self.get_logger().info("Connected to ESP32-CAM")
                except Exception as e:
                    self.get_logger().warn(f"Retrying connection... {e}")
                    time.sleep(1) # Wait before retrying
                    continue

            try:
                # Read larger chunks (32KB) to ensure we get full frames quickly
                chunk = stream.read(32768) 
                if not chunk:
                    stream.close()
                    stream = None
                    continue
                
                byte_buffer += chunk
                
                # Extract all complete JPEG frames currently in the buffer
                while True:
                    a = byte_buffer.find(b'\xff\xd8') # JPEG Start
                    b = byte_buffer.find(b'\xff\xd9') # JPEG End
                    
                    if a != -1 and b != -1:
                        jpg_bytes = byte_buffer[a:b+2]
                        byte_buffer = byte_buffer[b+2:]
                        
                        # Process and publish the frame
                        self.process_and_publish(jpg_bytes)
                    else:
                        # No complete frame left in buffer, go back to reading from stream
                        break
                        
                # Prevent buffer from growing infinitely if stream is corrupted
                if len(byte_buffer) > 1000000:
                    byte_buffer = bytes()

            except Exception as e:
                self.get_logger().error(f"Stream error: {e}")
                if stream: stream.close()
                stream = None

    def process_and_publish(self, jpg_bytes):
        msg_header = Header()
        msg_header.stamp = self.get_clock().now().to_msg()
        msg_header.frame_id = "esp32_camera_link"
        
        # 1. Publish Compressed (FAST - No CPU decoding needed)
        compressed_msg = CompressedImage()
        compressed_msg.header = msg_header
        compressed_msg.format = "jpeg"
        compressed_msg.data = jpg_bytes
        self.compressed_pub.publish(compressed_msg)
        
        # 2. Publish Raw (SLOW - Only do this if someone is actually watching)
        # This check saves MASSIVE CPU cycles
        if self.image_pub.get_subscription_count() > 0:
            np_arr = np.frombuffer(jpg_bytes, dtype=np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is not None:
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                img_msg.header = msg_header
                self.image_pub.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Esp32CamBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()