import requests
import time
import random

# Server details
SERVER_URL = "http://localhost:5000/upload"

# Bot configuration
MAP_SIZE = 10  # 10x10 grid
bot_id = random.randint(0, 1000)

# Initialize bot state
position = [MAP_SIZE // 2, MAP_SIZE // 2]  # Start in the middle of the map
local_map = [['.' for _ in range(MAP_SIZE)] for _ in range(MAP_SIZE)]

def get_imu_reading():
    # Fake IMU: Randomly decide direction to move
    directions = ['UP', 'DOWN', 'LEFT', 'RIGHT']
    return random.choice(directions)

def get_lidar_reading():
    # Fake LIDAR: Random distance to obstacle
    return random.uniform(0.5, 5.0)  # distance in meters

def update_local_map(x, y):
    # Mark current position
    for i in range(MAP_SIZE):
        for j in range(MAP_SIZE):
            if local_map[i][j] == 'B':
                local_map[i][j] = '.'  # Clear previous bot position
    local_map[y][x] = 'B'

def print_local_map():
    for row in local_map:
        print(' '.join(row))
    print("\n")

def self_go():
    global position
    direction = get_imu_reading()
    lidar = get_lidar_reading()

    print(f"[Bot {bot_id}] Moving {direction}, LIDAR reading: {lidar:.2f}m")

    # Move based on IMU reading
    if direction == 'UP' and position[1] > 0:
        position[1] -= 1
    elif direction == 'DOWN' and position[1] < MAP_SIZE - 1:
        position[1] += 1
    elif direction == 'LEFT' and position[0] > 0:
        position[0] -= 1
    elif direction == 'RIGHT' and position[0] < MAP_SIZE - 1:
        position[0] += 1

    # Update and print the local map
    update_local_map(position[0], position[1])
    print_local_map()

def send_to_server(sensor_data):
    try:
        response = requests.post(SERVER_URL, json=sensor_data, timeout=1)
        if response.status_code == 200:
            print(f"[Bot {bot_id}] Successfully sent data to server.")
        else:
            print(f"[Bot {bot_id}] Server error: {response.status_code}")
    except Exception as e:
        print(f"[Bot {bot_id}] Server unreachable. Switching to local movement. Error: {e}")

def main_loop():
    while True:
        # Simulate sensors
        imu = get_imu_reading()
        lidar = get_lidar_reading()

        sensor_data = {
            "bot_id": bot_id,
            "timestamp": time.time(),
            "imu_direction": imu,
            "lidar_distance": lidar,
            "position": {"x": position[0], "y": position[1]}
        }

        try:
            send_to_server(sensor_data)
        except:
            pass
        
        # Move locally no matter what
        self_go()
        
        time.sleep(1)

if __name__ == "__main__":
    main_loop()
