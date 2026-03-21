
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        
        # Parameters
        self.safety_distance = 0.5  # Meters
        self.side_clearance = 0.3   # Meters
        self.avoidance_speed = 0.1  # m/s
        self.turn_rate = 0.5        # rad/s

        # Subscribers
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel_raw', self.cmd_callback, 10) # Input from Camera
        
        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10) # Output to Pico 2

        self.last_raw_cmd = Twist()
        self.obstacle_detected = False
        self.avoidance_dir = 1.0 # 1 for Left, -1 for Right

    def scan_callback(self, msg):
        # Focus on the front arc (approx -30 to +30 degrees)
        # Assuming Lidar 0 is straight ahead. Adjust indices based on your Lidar's mounting.
        num_ranges = len(msg.ranges)
        front_arc = 30 # degrees
        center_idx = num_ranges // 2 # Check your Lidar orientation!
        
        # Get ranges for the front 60-degree cone
        samples = int((front_arc / 360.0) * num_ranges)
        front_ranges = msg.ranges[center_idx - samples : center_idx + samples]
        
        # Filter out 0.0 or inf
        valid_ranges = [r for r in front_ranges if msg.range_min < r < msg.range_max]
        
        if valid_ranges and min(valid_ranges) < self.safety_distance:
            self.obstacle_detected = True
            # Determine which side is clearer
            left_side = np.mean(msg.ranges[center_idx:center_idx+samples])
            right_side = np.mean(msg.ranges[center_idx-samples:center_idx])
            self.avoidance_dir = 1.0 if left_side > right_side else -1.0
        else:
            self.obstacle_detected = False

    def cmd_callback(self, msg):
        out_msg = Twist()
        
        if self.obstacle_detected:
            # Simple reactive side-step logic
            self.get_logger().warn("OBSTACLE DETECTED! Diverting...")
            out_msg.linear.x = self.avoidance_speed * 0.5 # Slow down
            out_msg.angular.z = self.turn_rate * self.avoidance_dir # Veer away
        else:
            # Pass through the camera-based command
            out_msg = msg
            
        self.cmd_pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()