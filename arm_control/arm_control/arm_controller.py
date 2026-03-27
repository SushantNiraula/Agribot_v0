#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
import time
import json
import busio

# --- Hardware Setup ---
try:
    import board
    i2c_bus = board.I2C()
except Exception:
    i2c_bus = busio.I2C(3, 2)

from adafruit_servokit import ServoKit

class ArmControllerNode(Node):
    def __init__(self):
        super().__init__('arm_controller')
        
        # Initialize Hardware
        self.kit = ServoKit(channels=16, i2c=i2c_bus)
        
        # State Tracking
        self.recorded_moves = []
        self.current_angles = {}
        
        # --- ROS 2 Interfaces ---
        
        # Subscribe to joint commands (The "Sliders")
        self.subscription = self.create_subscription(
            JointState,
            '/arm/joint_commands',
            self.joint_command_callback,
            10
        )
        
        # Create Services (The "Buttons")
        self.srv_play = self.create_service(Trigger, '/arm/play_sequence', self.play_callback)
        self.srv_clear = self.create_service(Trigger, '/arm/clear_sequence', self.clear_callback)
        self.srv_save = self.create_service(Trigger, '/arm/save_sequence', self.save_callback)
        
        self.get_logger().info("Arm Controller Node Started. Listening for commands...")

    # --- Hardware Control Logic ---

    def execute_hardware_move(self, motor, angle):
        if motor == 'A': 
            self.kit.servo[0].angle = angle
        elif motor == 'B':
            self.kit.servo[1].angle = angle
            self.kit.servo[5].angle = 180 - angle
        elif motor == 'C': 
            self.kit.servo[2].angle = angle
        elif motor == 'D': 
            self.kit.servo[3].angle = angle
        elif motor == 'E': 
            self.kit.servo[4].angle = angle

    def move_smoothly(self, motor, target_angle, speed_delay=0.015):
        target_angle = int(target_angle)
        
        if motor not in self.current_angles:
            self.execute_hardware_move(motor, target_angle)
            self.current_angles[motor] = target_angle
            return
            
        current = self.current_angles[motor]
        if current == target_angle:
            return
            
        step = 2 if target_angle > current else -2
        
        for angle in range(current, target_angle, step):
            self.execute_hardware_move(motor, angle)
            time.sleep(speed_delay)
            
        self.execute_hardware_move(motor, target_angle)
        self.current_angles[motor] = target_angle

    # --- ROS 2 Callbacks ---

    def joint_command_callback(self, msg):
        # JointState uses arrays: name=['A'], position=[90.0]
        for idx, motor_name in enumerate(msg.name):
            target_angle = msg.position[idx]
            
            # Move it smoothly
            self.move_smoothly(motor_name, target_angle, speed_delay=0.01)
            
            # Record it
            self.recorded_moves.append({'motor': motor_name, 'angle': int(target_angle)})
            
        self.get_logger().debug(f"Recorded moves: {len(self.recorded_moves)}")

    def play_callback(self, request, response):
        self.get_logger().info(f"Replaying {len(self.recorded_moves)} moves...")
        for move in self.recorded_moves:
            self.move_smoothly(move['motor'], move['angle'], speed_delay=0.015)
            
        response.success = True
        response.message = "Playback complete."
        return response

    def clear_callback(self, request, response):
        self.recorded_moves = []
        self.get_logger().info("Sequence memory cleared.")
        response.success = True
        response.message = "Memory cleared."
        return response

    def save_callback(self, request, response):
        try:
            with open('sequence.json', 'w') as f:
                json.dump(self.recorded_moves, f)
            self.get_logger().info("Sequence saved to sequence.json.")
            response.success = True
            response.message = "File saved successfully."
        except Exception as e:
            response.success = False
            response.message = f"Failed to save: {str(e)}"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ArmControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()