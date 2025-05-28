import numpy as np
import socket
import json
import threading

monitor_clients = []

# Initialize 50x50 map with -1
MAP_SIZE = 50
grid_map = np.full((MAP_SIZE, MAP_SIZE), -1)

# EKF parameters
state = np.zeros(3)  # x, y, theta
P = np.eye(3) * 0.1  # Covariance matrix

# Process and measurement noise
Q = np.diag([0.1, 0.1, 0.05])
R = np.diag([0.5, 0.5])  # LiDAR position noise

# Broadcast map to all monitor clients
def broadcast_map():
    map_data = grid_map.tolist()
    message = json.dumps({"map": map_data}).encode()
    #print(map_data)
    for client in monitor_clients:
        try:
            client.sendall(message)
        except:
            monitor_clients.remove(client)

def monitor_server(host='localhost', port=9001):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((host, port))
        s.listen()
        print(f"Monitor server listening on {host}:{port}")
        while True:
            conn, _ = s.accept()
            monitor_clients.append(conn)
            print("Monitor connected")

# Start monitor thread
threading.Thread(target=monitor_server, daemon=True).start()

# Sensor model: LiDAR data format: [(distance, angle), ...]
def ekf_predict(state, P, imu_data):
    dx, dy, dtheta = imu_data
    theta = state[2]

    # Predict next state
    state[0] += dx * np.cos(theta) - dy * np.sin(theta)
    state[1] += dx * np.sin(theta) + dy * np.cos(theta)
    state[2] += dtheta

    # Jacobian
    F = np.array([
        [1, 0, -dx * np.sin(theta) - dy * np.cos(theta)],
        [0, 1, dx * np.cos(theta) - dy * np.sin(theta)],
        [0, 0, 1]
    ])

    # Predict covariance
    P = F @ P @ F.T + Q
    return state, P

def ekf_update(state, P, lidar_data):
    for distance, angle in lidar_data:
        # Transform to global coordinates
        angle_world = state[2] + angle
        obs_x = state[0] + distance * np.cos(angle_world)
        obs_y = state[1] + distance * np.sin(angle_world)

        # Update map
        map_x, map_y = int(obs_x), int(obs_y)
        if 0 <= map_x < MAP_SIZE and 0 <= map_y < MAP_SIZE:
            grid_map[map_y, map_x] = 1  # Mark obstacle

        # Draw free space along the ray
        steps = int(distance)
        for step in range(steps):
            fx = state[0] + step * np.cos(angle_world)
            fy = state[1] + step * np.sin(angle_world)
            ix, iy = int(fx), int(fy)
            if 0 <= ix < MAP_SIZE and 0 <= iy < MAP_SIZE:
                if grid_map[iy, ix] == -1:  # Only update unknown
                    grid_map[iy, ix] = 0
    return state, P

# Simple TCP server to receive robot data
def run_server(host='localhost', port=9000):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((host, port))
        s.listen()
        print(f"Server listening on {host}:{port}")
        conn, addr = s.accept()
        with conn:
            print(f"Connected by {addr}")
            while True:
                data = conn.recv(4096)
                if not data:
                    break
                try:
                    message = json.loads(data.decode())
                    imu = message['imu']        # [dx, dy, dtheta]
                    lidar = message['lidar']    # [(distance, angle), ...]

                    global state, P
                    state, P = ekf_predict(state, P, imu)
                    state, P = ekf_update(state, P, lidar)

                    print("Updated Pose:", state)
                    broadcast_map()
                except Exception as e:
                    print("Error:", e)


if __name__ == "__main__":
    run_server()
