#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

import sys
import select
import termios
import tty

# --- Control Mapping ---
msg = """
Agribot Arm Teleop Control
---------------------------
Joint Controls (Press to move by 5 degrees):
  Base (A)     : 'u' to increase, 'j' to decrease
  Shoulder (B) : 'i' to increase, 'k' to decrease
  Elbow (C)    : 'o' to increase, 'l' to decrease
  Wrist (D)    : 'y' to increase, 'h' to decrease
  Gripper (E)  : 't' to open,     'g' to close

Sequence Memory Controls:
  'p' : Play recorded sequence
  'c' : Clear sequence memory
  'v' : Save sequence to file

CTRL-C or 'q' to quit
"""

class ArmTeleop(Node):
    def __init__(self):
        super().__init__('arm_teleop')
        
        # Publisher for the arm angles
        self.publisher_ = self.create_publisher(JointState, '/arm/joint_commands', 10)
        
        # Service Clients for memory functions
        self.cli_play = self.create_client(Trigger, '/arm/play_sequence')
        self.cli_clear = self.create_client(Trigger, '/arm/clear_sequence')
        self.cli_save = self.create_client(Trigger, '/arm/save_sequence')
        
        # Starting positions (90 degrees for all joints)
        self.angles = {'A': 90, 'B': 90, 'C': 90, 'D': 90, 'E': 90}
        self.step_size = 5

    def publish_joint(self, motor):
        msg = JointState()
        msg.name = [motor]
        msg.position = [float(self.angles[motor])]
        self.publisher_.publish(msg)
        # We print \r to overwrite the same line in the terminal
        print(f"\rMoved Motor {motor} to {self.angles[motor]}°      ", end='')

    def call_service(self, client, service_name):
        if not client.wait_for_service(timeout_sec=1.0):
            print(f"\rService {service_name} not available!      ", end='')
            return
        
        print(f"\rCalling {service_name}...                    ", end='')
        req = Trigger.Request()
        future = client.call_async(req)
        # We don't block to wait for the result here to keep the keyboard responsive

def get_key(settings):
    """Reads a single keypress from the terminal without needing to press Enter."""
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = ArmTeleop()

    print(msg)

    try:
        while rclpy.ok():
            key = get_key(settings)
            
            # --- Joint Controls ---
            if key == 'u':
                node.angles['A'] = min(180, node.angles['A'] + node.step_size)
                node.publish_joint('A')
            elif key == 'j':
                node.angles['A'] = max(0, node.angles['A'] - node.step_size)
                node.publish_joint('A')
                
            elif key == 'i':
                node.angles['B'] = min(180, node.angles['B'] + node.step_size)
                node.publish_joint('B')
            elif key == 'k':
                node.angles['B'] = max(0, node.angles['B'] - node.step_size)
                node.publish_joint('B')
                
            elif key == 'o':
                node.angles['C'] = min(180, node.angles['C'] + node.step_size)
                node.publish_joint('C')
            elif key == 'l':
                node.angles['C'] = max(0, node.angles['C'] - node.step_size)
                node.publish_joint('C')
                
            elif key == 'y':
                node.angles['D'] = min(180, node.angles['D'] + node.step_size)
                node.publish_joint('D')
            elif key == 'h':
                node.angles['D'] = max(0, node.angles['D'] - node.step_size)
                node.publish_joint('D')
                
            elif key == 't':
                node.angles['E'] = min(180, node.angles['E'] + node.step_size)
                node.publish_joint('E')
            elif key == 'g':
                node.angles['E'] = max(0, node.angles['E'] - node.step_size)
                node.publish_joint('E')

            # --- Service Controls ---
            elif key == 'p':
                node.call_service(node.cli_play, '/arm/play_sequence')
            elif key == 'c':
                node.call_service(node.cli_clear, '/arm/clear_sequence')
            elif key == 'v':
                node.call_service(node.cli_save, '/arm/save_sequence')

            # --- Quit ---
            elif key == 'q' or key == '\x03': # \x03 is Ctrl+C
                break

            # Spin once to process any pending ROS 2 callbacks in the background
            rclpy.spin_once(node, timeout_sec=0)

    except Exception as e:
        print(f"Error: {e}")

    finally:
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()