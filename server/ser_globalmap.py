# server.py
from flask import Flask, request, jsonify
import numpy as np

app = Flask(__name__)

# Create global map
global_map = np.full((100, 100), '.', dtype=str)

@app.route('/upload', methods=['POST'])
def upload():
    data = request.json
    bot_id = data['bot_id']
    pos = data['position']
    updates = data.get('local_map', [])

    print(f"[Server] Received data from Bot {bot_id} at {pos}")

    # Update global map with bot's local map
    if updates:
        update_global_map(pos, updates)

    # Send back a fresh 10x10 local map centered at bot
    local_map = get_local_map(pos)
    return jsonify({"local_map": local_map})

@app.route('/global_map', methods=['GET'])
def get_global_map():
    return jsonify({"global_map": global_map.tolist()})

def update_global_map(pos, updates):
    x_center, y_center = pos['x'], pos['y']
    for dy, row in enumerate(updates):
        for dx, value in enumerate(row):
            gx = x_center - 5 + dx
            gy = y_center - 5 + dy
            if 0 <= gx < 100 and 0 <= gy < 100:
                if value != '.':  # Don't overwrite unless new data
                    global_map[gy][gx] = value

def get_local_map(pos):
    x_center, y_center = pos['x'], pos['y']
    local = []
    for dy in range(-5, 5):
        row = []
        for dx in range(-5, 5):
            gx = x_center + dx
            gy = y_center + dy
            if 0 <= gx < 100 and 0 <= gy < 100:
                row.append(global_map[gy][gx])
            else:
                row.append('X')  # 'X' = wall / out of bounds
        local.append(row)
    return local

if __name__ == '__main__':
    app.run(port=5000, debug=True)
