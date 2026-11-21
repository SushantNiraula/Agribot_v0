import busio
import time
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
recorded_moves = [] # List of dictionaries: {'motor': 'A', 'angle': 90}

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
        button { width: 45%; height: 50px; margin: 5px; cursor: pointer; font-weight: bold; border-radius: 5px; border: none; }
        .btn-play { background: #2ecc71; color: white; }
        .btn-clear { background: #e74c3c; color: white; }
    </style>
</head>
<body>
    <h1>Agribot Arm Control</h1>
    
    <div class="slider-card">
        <p>Base (A): <span id="valA">90</span>°</p>
        <input type="range" min="0" max="180" value="90" oninput="update('A', this.value)">
        
        <p>Shoulder (B): <span id="valB">90</span>°</p>
        <input type="range" min="0" max="180" value="90" oninput="update('B', this.value)">

        <p>Elbow (C): <span id="valC">90</span>°</p>
        <input type="range" min="0" max="180" value="90" oninput="update('C', this.value)">

        <p>Wrist (D): <span id="valD">90</span>°</p>
        <input type="range" min="0" max="180" value="90" oninput="update('D', this.value)">

        <p>Gripper (E): <span id="valE">90</span>°</p>
        <input type="range" min="0" max="180" value="90" oninput="update('E', this.value)">
    </div>

    <div class="slider-card">
        <h3>Memory: <span id="moveCount">0</span> steps</h3>
        <button class="btn-play" onclick="play()">PLAY SEQUENCE</button>
        <button class="btn-clear" onclick="clearMoves()">CLEAR MEMORY</button>
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
        socket.on('count_update', function(data) {
            document.getElementById('moveCount').innerText = data.count;
        });
    </script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML)

@socketio.on('slider_move')
def handle_move(data):
    motor = data['motor']
    angle = data['angle']
    
    # Execute the move
    execute_hardware_move(motor, angle)
    
    # SAVE the move to memory
    recorded_moves.append({'motor': motor, 'angle': angle})
    socketio.emit('count_update', {'count': len(recorded_moves)})

@socketio.on('clear_sequence')
def clear_sequence():
    global recorded_moves
    recorded_moves = []
    print("Memory Cleared")

@socketio.on('play_sequence')
def play_sequence():
    print(f"Replaying {len(recorded_moves)} moves...")
    for move in recorded_moves:
        execute_hardware_move(move['motor'], move['angle'])
        time.sleep(0.05) # Small delay for smooth playback

def execute_hardware_move(motor, angle):
    if motor == 'A': kit.servo[0].angle = angle
    elif motor == 'B':
        kit.servo[1].angle = angle
        kit.servo[5].angle = 180 - angle # Sync dual shoulder
    elif motor == 'C': kit.servo[2].angle = angle
    elif motor == 'D': kit.servo[3].angle = angle
    elif motor == 'E': kit.servo[4].angle = angle

if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=5000)
