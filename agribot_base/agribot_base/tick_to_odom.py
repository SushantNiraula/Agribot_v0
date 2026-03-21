#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math

class TickToOdom(Node):
    def __init__(self):
        super().__init__('tick_to_odom')
        
        # Subscriptions to Pico 2 micro-ROS topics
        self.sub_l = self.create_subscription(Int32, '/left_ticks', self.l_cb, 10)
        self.sub_r = self.create_subscription(Int32, '/right_ticks', self.r_cb, 10)
        
        # Publisher for Odometry
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        
        # Internal state
        self.prev_l_ticks = 0
        self.prev_r_ticks = 0
        self.first_read = True
        
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        self.last_time = self.get_clock().now()
        
        # --- ROBOT PARAMETERS (ADJUST THESE) ---
        self.wheel_radius = 0.055  # 55mm radius / 110mm diameter
        self.wheel_base = 0.5    # 35cm between left and right wheels
        self.ticks_per_rev = 280.0 # Ticks per full wheel rotation
        # ---------------------------------------
        
        self.meters_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev
        
        # Timer to publish odom at 20Hz
        self.timer = self.create_timer(0.05, self.update_and_publish)

        self.current_l_ticks = 0
        self.current_r_ticks = 0

    def l_cb(self, msg):
        self.current_l_ticks = msg.data

    def r_cb(self, msg):
        self.current_r_ticks = msg.data

    def update_and_publish(self):
        if self.first_read:
            self.prev_l_ticks = self.current_l_ticks
            self.prev_r_ticks = self.current_r_ticks
            self.first_read = False
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        if dt <= 0:
            return

        # Calculate tick deltas
        delta_l = self.current_l_ticks - self.prev_l_ticks
        delta_r = self.current_r_ticks - self.prev_r_ticks

        # Save current ticks for next iteration
        self.prev_l_ticks = self.current_l_ticks
        self.prev_r_ticks = self.current_r_ticks

        # Convert ticks to meters traveled by each wheel
        d_l = delta_l * self.meters_per_tick
        d_r = delta_r * self.meters_per_tick

        # Differential drive kinematics
        d_center = (d_l + d_r) / 2.0
        d_theta = (d_r - d_l) / self.wheel_base

        # Calculate velocities
        v_x = d_center / dt
        v_theta = d_theta / dt

        # Update pose estimation
        self.th += d_theta
        self.x += d_center * math.cos(self.th)
        self.y += d_center * math.sin(self.th)

        # Create Odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Set Pose
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Convert Theta to Quaternion
        odom.pose.pose.orientation = self.yaw_to_quaternion(self.th)

        # Set Twist (Velocity)
        odom.twist.twist.linear.x = v_x
        odom.twist.twist.angular.z = v_theta

        # Publish
        self.pub_odom.publish(odom)
        self.last_time = current_time

    def yaw_to_quaternion(self, yaw):
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

def main(args=None):
    rclpy.init(args=args)
    node = TickToOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()