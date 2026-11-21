import busio
import time
import json
from adafruit_servokit import ServoKit

# --- Hardware Setup ---
try:
    import board
    i2c_bus = board.I2C()
except Exception:
    i2c_bus = busio.I2C(3, 2)

kit = ServoKit(channels=16, i2c=i2c_bus)

# --- Your Custom Home Position ---
HOME_ANGLES = {
    'A': 170,  # Base
    'B': 23,   # Shoulder
    'C': 170,  # Elbow
    'D': 170,  # Wrist
    'E': 90    # Gripper
}

# We need to remember where the arm currently is so we can step slowly
current_angles = {}

def execute_hardware_move(motor, angle):
    if motor == 'A': kit.servo[0].angle = angle
    elif motor == 'B':
        kit.servo[1].angle = angle
        kit.servo[5].angle = 180 - angle # Dual shoulder sync
    elif motor == 'C': kit.servo[2].angle = angle
    elif motor == 'D': kit.servo[3].angle = angle
    elif motor == 'E': kit.servo[4].angle = angle

def move_smoothly(motor, target_angle, speed_delay=0.015):
    target_angle = int(target_angle)
    
    # If it's the very first time the Pi turned on, we don't know the physical angle.
    # We have to jump to establish our starting point.
    if motor not in current_angles:
        execute_hardware_move(motor, target_angle)
        current_angles[motor] = target_angle
        return
        
    current = current_angles[motor]
    
    if current == target_angle:
        return # Already there!
        
    # Decide direction: +2 degrees if moving up, -2 degrees if moving down
    step = 2 if target_angle > current else -2
    
    # Step slowly toward the target
    for angle in range(current, target_angle, step):
        execute_hardware_move(motor, angle)
        time.sleep(speed_delay) # Pause between steps
        
    # Make sure it hits the exact final angle (handles the "odd 1 degree" remainder)
    execute_hardware_move(motor, target_angle)
    current_angles[motor] = target_angle

def go_home():
    print("\n>>> Moving to HOME position smoothly...")
    for motor, angle in HOME_ANGLES.items():
        # I set the speed_delay a tiny bit slower (0.025) so homing looks very deliberate
        move_smoothly(motor, angle, speed_delay=0.025) 
        time.sleep(0.2) # Small pause between joints
    print(">>> Arrived at HOME.")

print("--- Agribot Task Runner ---")

try:
    # 1. Open and read the saved sequence
    with open('sequence.json', 'r') as f:
        moves = json.load(f)
        
    print(f"Loaded {len(moves)} steps from memory.")
    
    # 2. Go to Home Position first
    go_home()
    
    # 3. Wait for safety
    print("\nStarting automated loop in 3 seconds. CLEAR YOUR HANDS!")
    time.sleep(3) 
    
    # 4. Run the sequence continuously
    while True:
        print("Running sequence...")
        for move in moves:
            # We use our new smooth function instead of raw jumps!
            move_smoothly(move['motor'], move['angle'], speed_delay=0.015) 
            
        print("Sequence complete. Resetting to HOME for next loop...")
        go_home()
        time.sleep(2) # Wait 2 seconds before repeating the whole task

except FileNotFoundError:
    print("Error: 'sequence.json' not found. Run the web server and SAVE a sequence first!")
except KeyboardInterrupt:
    print("\nAutomation stopped safely.")
    go_home()
