#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
import time
import json
import os
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
        
        # 1. Initialize Hardware
        self.kit = ServoKit(channels=16, i2c=i2c_bus)
        
        # 2. State Tracking
        self.recorded_moves = []
        self.current_angles = {}
        self.last_angles_file = 'last_angles.json'
        
        # Define the exact Safe Home Positions you provided
        self.home_positions = {
            'A': 90, 
            'B': 131, 
            'C': 180, 
            'D': 153, 
            'E': 85
        }

        # 3. Load the last known positions before homing!
        self.load_last_angles()
        
        # 4. Home the hardware smoothly upon startup
        self.get_logger().info("Homing arm hardware to safe positions smoothly...")
        self.hardware_go_home()
        
        # Clear any recorded moves from the homing process 
        self.recorded_moves = [] 
        
        # --- ROS 2 Interfaces ---
        self.subscription = self.create_subscription(
            JointState,
            '/arm/joint_commands',
            self.joint_command_callback,
            10
        )
        
        self.srv_play = self.create_service(Trigger, '/arm/play_sequence', self.play_callback)
        self.srv_clear = self.create_service(Trigger, '/arm/clear_sequence', self.clear_callback)
        self.srv_save = self.create_service(Trigger, '/arm/save_sequence', self.save_callback)
        
        self.get_logger().info("Arm Controller Node Started. Listening for commands...")

    # --- Hardware Control Logic ---

    def load_last_angles(self):
        """Loads the last known angles from a file so the robot doesn't snap on boot."""
        if os.path.exists(self.last_angles_file):
            try:
                with open(self.last_angles_file, 'r') as f:
                    self.current_angles = json.load(f)
                self.get_logger().info(f"Loaded previous angles from memory: {self.current_angles}")
            except Exception as e:
                self.get_logger().error(f"Could not load last angles: {e}")
                self.set_default_start_angles()
        else:
            self.set_default_start_angles()

    def set_default_start_angles(self):
        """If no memory exists (first run ever), assume the arm is resting at 90s to allow interpolation."""
        self.get_logger().info("No previous memory found. Assuming all motors are at 90 degrees.")
        self.current_angles = {'A': 90, 'B': 90, 'C': 90, 'D': 90, 'E': 90}

    def save_last_angles(self):
        """Saves current angles to a file so we can resume smoothly on next boot."""
        try:
            with open(self.last_angles_file, 'w') as f:
                json.dump(self.current_angles, f)
        except Exception as e:
            self.get_logger().debug(f"Failed to save last angles memory: {e}")

    def hardware_go_home(self):
        """Moves all servos to their home position smoothly."""
        for motor, angle in self.home_positions.items():
            self.move_smoothly(motor, angle, speed_delay=0.015)

    def execute_hardware_move(self, motor, angle):
        """Direct connection to the PCA9685 board."""
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
            
        current = self.current_angles[motor]
        if current == target_angle:
            return
            
        step = 2 if target_angle > current else -2
        
        # Smoothly step towards target
        for angle in range(current, target_angle, step):
            self.execute_hardware_move(motor, angle)
            time.sleep(speed_delay)
            
        # Ensure final position is hit perfectly
        self.execute_hardware_move(motor, target_angle)
        self.current_angles[motor] = target_angle
        
        # Save position quietly in the background
        self.save_last_angles()

    # --- ROS 2 Callbacks ---

    def joint_command_callback(self, msg):
        for idx, motor_name in enumerate(msg.name):
            target_angle = msg.position[idx]
            self.move_smoothly(motor_name, target_angle, speed_delay=0.01)
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