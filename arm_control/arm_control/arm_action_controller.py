#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState, Image
from std_srvs.srv import Trigger
from agribot_interfaces.action import CropInspection  # Our new Action!
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import json
import os
import busio
import urllib.request

# --- Hardware Setup ---
try:
    import board
    i2c_bus = board.I2C()
except Exception:
    i2c_bus = busio.I2C(3, 2)

from adafruit_servokit import ServoKit
import RPi.GPIO as GPIO

# --- GPIO Configuration ---
PUMP_IN1 = 17
PUMP_IN2 = 27
SOIL_SENSOR_D0 = 22

GPIO.setmode(GPIO.BCM)
GPIO.setup(PUMP_IN1, GPIO.OUT)
GPIO.setup(PUMP_IN2, GPIO.OUT)
GPIO.setup(SOIL_SENSOR_D0, GPIO.IN)

class ArmControllerNode(Node):
    def __init__(self):
        super().__init__('arm_action_controller')
        self.bridge = CvBridge()
        self.kit = ServoKit(channels=16, i2c=i2c_bus)
        self.esp_ip = '10.63.158.232'
        
        # ==========================================================
        # WAYPOINTS (Kept intact as requested)
        # ==========================================================
        self.home_pos   = {'A': 90, 'B': 131, 'C': 161, 'D': 160, 'E': 0}
        self.look_pos   = {'A': 113, 'B': 154, 'C': 174, 'D': 160, 'E': 0} 
        self.image_pos  = {'A': 113, 'B': 78,  'C': 122, 'D': 158, 'E': 0} 
        self.deploy_pos = {'A': 113, 'B': 78,  'C': 122, 'D': 158, 'E': 103}  
        self.insert_pos = {'A': 113, 'B': 24,  'C': 104, 'D': 158, 'E': 103}  
        # ==========================================================
        
        self.current_angles = {}
        self.last_angles_file = 'last_angles.json'
        
        # Vision State
        self.is_inspecting = False
        self.is_centering = False
        self.current_visual_error_x = 0  
        self.current_visual_error_y = 0 
        self.blob_detected = False

        self.load_last_angles()
        self.get_logger().info("Homing arm hardware...")
        self.move_simultaneously(self.home_pos) 
        
        # ROS 2 Interfaces
        # We use a ReentrantCallbackGroup so the action server and vision subscriber don't block each other
        self.cb_group = ReentrantCallbackGroup()
        
        self.subscription = self.create_subscription(JointState, '/arm/joint_commands', self.joint_command_callback, 10, callback_group=self.cb_group)
        self.cam_sub = self.create_subscription(Image, '/esp32cam/image_raw', self.camera_callback, 10, callback_group=self.cb_group)
        self.cam_pub = self.create_publisher(Image, '/esp32/arm_action', 10)
        self.save_img_client = self.create_client(Trigger, '/save_crop_image', callback_group=self.cb_group)
        
        # --- THE NEW ACTION SERVER ---
        self.action_server = ActionServer(
            self,
            CropInspection,
            '/arm/inspect_crop',
            self.execute_callback,
            callback_group=self.cb_group
        )
        self.get_logger().info("Action Server Ready. Waiting for Goal...")

    def load_last_angles(self):
        if os.path.exists(self.last_angles_file):
            try:
                with open(self.last_angles_file, 'r') as f:
                    self.current_angles = json.load(f)
            except Exception:
                self.current_angles = self.home_pos.copy()
        else:
            self.current_angles = self.home_pos.copy()

    def save_last_angles(self):
        try:
            with open(self.last_angles_file, 'w') as f:
                json.dump(self.current_angles, f)
        except Exception:
            pass

    def execute_hardware_move(self, motor, angle):
        # FIX: Hard clamp the angle between 0 and 180 to prevent I2C crashes
        angle = max(0, min(180, int(angle)))
        
        motor_mapping = {'A': [0], 'B': [1, 5], 'C': [2], 'D': [3], 'E': [4]}
        max_retries = 3
        for attempt in range(max_retries):
            try:
                for idx in motor_mapping[motor]:
                    send_angle = 180 - angle if idx == 5 else angle
                    send_angle = max(0, min(180, send_angle)) # Double check clamp
                    self.kit.servo[idx].angle = send_angle
                break 
            except OSError:
                if attempt == max_retries - 1:
                    self.get_logger().error(f"I2C Bus Dead. Failed to move {motor}.")
                else:
                    time.sleep(0.02) 

    def move_simultaneously(self, target_dict, speed_delay=0.015):
        active = True
        while active:
            active = False
            for motor, target in target_dict.items():
                # Clamp the target limit
                target = int(max(0, min(180, target)))
                
                if motor not in self.current_angles:
                    self.current_angles[motor] = 90
                current = self.current_angles[motor]
                
                if current != target:
                    active = True
                    step = 1 if target > current else -1
                    new_angle = current + step
                    self.execute_hardware_move(motor, new_angle)
                    self.current_angles[motor] = new_angle
            if active:
                time.sleep(speed_delay)
        self.save_last_angles()

    def set_flashlight(self, intensity):
        try:
            url = f"http://{self.esp_ip}/control?var=led_intensity&val={intensity}"
            urllib.request.urlopen(url, timeout=3)
        except Exception:
            pass

    def check_soil_and_water(self, publish_feedback):
        publish_feedback("Reading soil moisture...")
        time.sleep(5.0) 
        
        is_dry = GPIO.input(SOIL_SENSOR_D0) 
        if is_dry:
            publish_feedback("Soil is DRY. Watering plant for 3 seconds...")
            GPIO.output(PUMP_IN1, GPIO.HIGH)
            GPIO.output(PUMP_IN2, GPIO.LOW)
            time.sleep(3.0)
            GPIO.output(PUMP_IN1, GPIO.LOW) 
            publish_feedback("Watering complete.")
        else:
            publish_feedback("Soil is adequately wet. No water needed.")

    def camera_callback(self, msg):
        if not self.is_inspecting:
            return
            
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = cv_image.shape[:2]
        image_center_x = w // 2
        image_center_y = h // 2

        if self.is_centering:
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            lower_green = np.array([35, 50, 50])
            upper_green = np.array([85, 255, 255])
            mask = cv2.inRange(hsv, lower_green, upper_green)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest_contour) > 500: 
                    self.blob_detected = True
                    M = cv2.moments(largest_contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        self.current_visual_error_x = image_center_x - cx
                        self.current_visual_error_y = image_center_y - cy
                else:
                    self.blob_detected = False
            else:
                self.blob_detected = False

        self.cam_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8'))

    def joint_command_callback(self, msg):
        targets = {}
        for idx, motor_name in enumerate(msg.name):
            targets[motor_name] = msg.position[idx]
        self.move_simultaneously(targets, speed_delay=0.01)

    def align_to_target(self, publish_feedback):
        publish_feedback("Activating 2D Visual Alignment...")
        self.is_centering = True
        centering_attempts = 0
        
        while self.is_centering and centering_attempts < 60: 
            if not self.blob_detected:
                centering_attempts += 1
                time.sleep(0.1)
                continue
                
            error_x = self.current_visual_error_x
            error_y = self.current_visual_error_y
            current_a = self.current_angles['A']
            current_b = self.current_angles['B']
            
            needs_x_nudge = abs(error_x) > 20
            needs_y_nudge = abs(error_y) > 20
            
            if not needs_x_nudge and not needs_y_nudge: 
                publish_feedback("Target Perfectly Aligned!")
                break
                
            if needs_x_nudge:
                new_a = current_a + 1 if error_x > 0 else current_a - 1
                self.move_simultaneously({'A': new_a})
                
            if needs_y_nudge:
                new_b = current_b + 1 if error_y > 0 else current_b - 1
                self.move_simultaneously({'B': new_b})
                
            centering_attempts += 1
            time.sleep(0.1) 
            
        self.is_centering = False
        time.sleep(1.0) 

    # --- THE NEW ACTION CALLBACK ---
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = CropInspection.Feedback()
        
        # Helper function to log and send live feedback
        def publish_feedback(msg_text):
            self.get_logger().info(msg_text)
            feedback_msg.current_state = msg_text
            goal_handle.publish_feedback(feedback_msg)

        self.is_inspecting = True 
        publish_feedback("=== PHASE 1: IMAGE COLLECTION ===")
        
        self.set_flashlight(200)
        time.sleep(2.0)
        
        publish_feedback("Moving to Look Position...")
        self.move_simultaneously(self.look_pos, speed_delay=0.015)
        self.align_to_target(publish_feedback)
            
        publish_feedback("Lowering for Image Capture...")
        img_target = self.current_angles.copy()
        img_target.update({'B': self.image_pos['B'], 'C': self.image_pos['C'], 'D': self.image_pos['D']})
        self.move_simultaneously(img_target)
        time.sleep(2.0) 
        
        publish_feedback("Triggering camera to save image...")
        if self.save_img_client.wait_for_service(timeout_sec=1.0):
            self.save_img_client.call_async(Trigger.Request())
        time.sleep(2.0)
        
        publish_feedback("Returning Home...")
        self.move_simultaneously(self.home_pos)
        
        publish_feedback("=== PHASE 2: SOIL PROBING ===")
        publish_feedback("Moving to Look Position...")
        self.move_simultaneously(self.look_pos, speed_delay=0.015)
        self.align_to_target(publish_feedback)
        
        publish_feedback("Deploying Soil Probe...")
        deploy_target = self.current_angles.copy() 
        deploy_target.update({'E': self.deploy_pos['E'], 'D': self.deploy_pos['D'], 'C': self.deploy_pos['C'], 'B': self.deploy_pos['B']})
        self.move_simultaneously(deploy_target)
        time.sleep(0.5)
        
        publish_feedback("Inserting into soil...")
        insert_target = self.current_angles.copy() 
        insert_target.update({'B': self.insert_pos['B'], 'C': self.insert_pos['C'], 'D': self.insert_pos['D']})
        self.move_simultaneously(insert_target, speed_delay=0.02)
        
        self.check_soil_and_water(publish_feedback)
        
        publish_feedback("Extracting and Homing...")
        self.move_simultaneously(self.home_pos)
        
        self.set_flashlight(0)
        self.is_inspecting = False 
        
        # Action is finished successfully!
        goal_handle.succeed()
        result = CropInspection.Result()
        result.success = True
        result.message = "Routine Completed Successfully!"
        return result

def main(args=None):
    rclpy.init(args=args)
    node = ArmControllerNode()
    
    # We use a MultiThreadedExecutor so the ActionServer can run alongside the Camera Subscriber
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup() 
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()