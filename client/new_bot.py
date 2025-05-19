import socket
import json
import random
import time
import math

HOST = 'localhost'
PORT = 9000

def generate_random_imu():
    # Simulate small robot movement: [dx, dy, dtheta]
    dx = random.uniform(0.1, 0.3)
    dy = random.uniform(-0.05, 0.05)
    dtheta = random.uniform(-0.1, 0.1)
    return [dx, dy, dtheta]

def generate_random_lidar():
    # Simulate 10 random LiDAR readings (distance, angle)
    lidar_data = []
    for _ in range(10):
        distance = random.uniform(2.0, 10.0)  # 2m to 10m
        angle = random.uniform(-math.pi / 2, math.pi / 2)  # ±90°
        lidar_data.append([distance, angle])
    return lidar_data

def run_bot():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        print("Bot connected to server.")

        while True:
            imu = generate_random_imu()
            lidar = generate_random_lidar()

            data = {
                "imu": imu,
                "lidar": lidar
            }

            s.sendall(json.dumps(data).encode())
            print(f"Sent data: IMU={imu}, LiDAR={len(lidar)} readings")

            time.sleep(1)  # Send once per second

if __name__ == "__main__":
    run_bot()
