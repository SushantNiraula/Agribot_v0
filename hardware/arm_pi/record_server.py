import busio
import time
import json
from flask import Flask, render_template_string
from flask_socketio import SocketIO
from adafruit_servokit import ServoKit

# --- Hardware Setup ---
try:
    import board
    i2c_bus = board.I2C()
except Exception:
    i2c_bus = busio.I2C(3, 2)

kit = ServoKit(channels=16, i2c=i2c_bus)
app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

# --- Recording Memory ---
recorded_moves = [] 
current_angles = {} # Remembers where the arm is right now

HTML = """
<!DOCTYPE html>
<html>
<head>
    <title>Agribot Pro Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { text-align:center; font-family:sans-serif; background:#222; color:white; padding:20px; }
        .slider-card { background:#333; padding:15px; border-radius:10px; margin:10px auto; max-width:400px; border:1px solid #444; }
        input[type=range] { width: 100%; height: 25px; }
        button { width: 30%; height: 50px; margin: 5px; cursor: pointer; font-weight: bold; border-radius: 5px; border: none; }
        .btn-play { background: #2ecc71; color: white; }
        .btn-clear { background: #e74c3c; color: white; }
        .btn-save { background: #3498db; color: white; }
    </style>
</head>
<body>
    <h1>Agribot Arm Control</h1>
    
    <div class="slider-card">
        <p>Base (A): <span id="valA">90</span>°</p>
        <input type="range" min="10" max="170" value="90" oninput="update('A', this.value)">
        
        <p>Shoulder (B): <span id="valB">90</span>°</p>
        <input type="range" min="10" max="170" value="90" oninput="update('B', this.value)">

        <p>Elbow (C): <span id="valC">90</span>°</p>
        <input type="range" min="10" max="170" value="90" oninput="update('C', this.value)">

        <p>Wrist (D): <span id="valD">90</span>°</p>
        <input type="range" min="10" max="170" value="90" oninput="update('D', this.value)">

        <p>Gripper (E): <span id="valE">90</span>°</p>
        <input type="range" min="10" max="170" value="90" oninput="update('E', this.value)">
    </div>

    <div class="slider-card">
        <h3>Memory: <span id="moveCount">0</span> steps</h3>
        <button class="btn-play" onclick="play()">PLAY</button>
        <button class="btn-clear" onclick="clearMoves()">CLEAR</button>
        <button class="btn-save" onclick="saveToFile()">SAVE</button>
    </div>
    
    <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.1/socket.io.js"></script>
    <script>
        var socket = io();
        function update(motor, val) {
            document.getElementById('val' + motor).innerText = val;
            socket.emit('slider_move', {motor: motor, angle: parseInt(val)});
        }
        function play() { socket.emit('play_sequence'); }
        function clearMoves() { 
            socket.emit('clear_sequence'); 
            document.getElementById('moveCount').innerText = "0";
        }
        function saveToFile() { socket.emit('save_sequence'); }
        
        socket.on('count_update', function(data) {
            document.getElementById('moveCount').innerText = data.count;
        });
        socket.on('alert_msg', function(data) {
            alert(data.message);
        });
    </script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML)

def execute_hardware_move(motor, angle):
    if motor == 'A': kit.servo[0].angle = angle
    elif motor == 'B':
        kit.servo[1].angle = angle
        kit.servo[5].angle = 180 - angle
    elif motor == 'C': kit.servo[2].angle = angle
    elif motor == 'D': kit.servo[3].angle = angle
    elif motor == 'E': kit.servo[4].angle = angle

def move_smoothly(motor, target_angle, speed_delay=0.015):
    target_angle = int(target_angle)
    
    # Initialize start position if it's the first move
    if motor not in current_angles:
        execute_hardware_move(motor, target_angle)
        current_angles[motor] = target_angle
        return
        
    current = current_angles[motor]
    
    if current == target_angle:
        return
        
    # Step 2 degrees at a time
    step = 2 if target_angle > current else -2
    
    for angle in range(current, target_angle, step):
        execute_hardware_move(motor, angle)
        time.sleep(speed_delay)
        
    # Snap to the final exact angle
    execute_hardware_move(motor, target_angle)
    current_angles[motor] = target_angle

@socketio.on('slider_move')
def handle_move(data):
    motor = data['motor']
    angle = data['angle']
    
    # We now use move_smoothly for real-time control too!
    # A slightly faster delay (0.01) keeps it feeling responsive to your finger
    move_smoothly(motor, angle, speed_delay=0.01) 
    
    recorded_moves.append({'motor': motor, 'angle': angle})
    
    # Throttle the UI update so the browser doesn't lag while sliding
    if len(recorded_moves) % 5 == 0:
        socketio.emit('count_update', {'count': len(recorded_moves)})

@socketio.on('clear_sequence')
def clear_sequence():
    global recorded_moves
    recorded_moves = []
    socketio.emit('count_update', {'count': 0})
    print("Memory Cleared")

@socketio.on('play_sequence')
def play_sequence():
    print(f"Replaying {len(recorded_moves)} moves smoothly...")
    for move in recorded_moves:
        # We also use move_smoothly for the web UI playback
        move_smoothly(move['motor'], move['angle'], speed_delay=0.015)

@socketio.on('save_sequence')
def save_sequence():
    with open('sequence.json', 'w') as f:
        json.dump(recorded_moves, f)
    print("Sequence saved to file!")
    socketio.emit('alert_msg', {'message': 'Sequence successfully saved to Pi!'})

if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=5000)
