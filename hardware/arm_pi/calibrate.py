import busio
import time
from adafruit_servokit import ServoKit

# Manual I2C setup for your Ubuntu Pi 4
try:
    import board
    i2c_bus = board.I2C()
except Exception:
    i2c_bus = busio.I2C(3, 2)

kit = ServoKit(channels=16, i2c=i2c_bus)

print("--- Agribot Calibration Tool ---")
print("Press Ctrl+C to exit.")

try:
    while True:
        try:
            port_input = input("\nEnter PCA9685 Port (0-15): ")
            angle_input = input("Enter Angle (0-180): ")
            
            port = int(port_input)
            angle = int(angle_input)
            
            # Safety constrain
            angle = max(0, min(180, angle))
            port = max(0, min(15, port))
            
            print(f">>> Moving Port {port} to {angle}°")
            kit.servo[port].angle = angle
            
        except ValueError:
            print("Please enter valid numbers.")
            
except KeyboardInterrupt:
    print("\nCalibration stopped.")
