import requests
import time
import random
import pygame
import threading

# Server details
SERVER_URL = "http://localhost:5000/upload"

# Bot configuration
MAP_SIZE = 10  # 10x10 grid
CELL_SIZE = 50  # Each cell is 50x50 pixels
WINDOW_SIZE = MAP_SIZE * CELL_SIZE

bot_id = random.randint(0, 1000)
position = [MAP_SIZE // 2, MAP_SIZE // 2]  # Start in center

local_map = [['.' for _ in range(MAP_SIZE)] for _ in range(MAP_SIZE)]

# Initialize Pygame
pygame.init()
screen = pygame.display.set_mode((WINDOW_SIZE, WINDOW_SIZE))
pygame.display.set_caption(f"Bot {bot_id} Local Map")
clock = pygame.time.Clock()

def get_imu_reading():
    directions = ['UP', 'DOWN', 'LEFT', 'RIGHT']
    return random.choice(directions)

def get_lidar_reading():
    return random.uniform(0.5, 5.0)

def update_local_map(x, y):
    # Clear old bot position
    for i in range(MAP_SIZE):
        for j in range(MAP_SIZE):
            if local_map[i][j] == 'B':
                local_map[i][j] = '.'
    local_map[y][x] = 'B'

def draw_map():
    screen.fill((255, 255, 255))  # White background
    for y in range(MAP_SIZE):
        for x in range(MAP_SIZE):
            rect = pygame.Rect(x*CELL_SIZE, y*CELL_SIZE, CELL_SIZE, CELL_SIZE)
            if local_map[y][x] == 'B':
                pygame.draw.rect(screen, (0, 128, 255), rect)  # Bot = Blue
            else:
                pygame.draw.rect(screen, (200, 200, 200), rect, 1)  # Grid lines
    pygame.display.flip()

def self_go():
    global position
    direction = get_imu_reading()
    lidar = get_lidar_reading()

    print(f"[Bot {bot_id}] Moving {direction}, LIDAR: {lidar:.2f}m")

    if direction == 'UP' and position[1] > 0:
        position[1] -= 1
    elif direction == 'DOWN' and position[1] < MAP_SIZE - 1:
        position[1] += 1
    elif direction == 'LEFT' and position[0] > 0:
        position[0] -= 1
    elif direction == 'RIGHT' and position[0] < MAP_SIZE - 1:
        position[0] += 1

    update_local_map(position[0], position[1])

def send_to_server(sensor_data):
    try:
        response = requests.post(SERVER_URL, json=sensor_data, timeout=1)
        if response.status_code == 200:
            print(f"[Bot {bot_id}] Sent data to server.")
        else:
            print(f"[Bot {bot_id}] Server error: {response.status_code}")
    except Exception as e:
        print(f"[Bot {bot_id}] Server unreachable: {e}")

def bot_main_loop():
    while True:
        imu = get_imu_reading()
        lidar = get_lidar_reading()

        sensor_data = {
            "bot_id": bot_id,
            "timestamp": time.time(),
            "imu_direction": imu,
            "lidar_distance": lidar,
            "position": {"x": position[0], "y": position[1]}
        }

        send_to_server(sensor_data)
        self_go()
        time.sleep(1)

def pygame_loop():
    running = True
    while running:
        clock.tick(10)  # 10 FPS
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        draw_map()

    pygame.quit()
    exit()

if __name__ == "__main__":
    # Start bot logic in a separate thread
    bot_thread = threading.Thread(target=bot_main_loop, daemon=True)
    bot_thread.start()

    # Start Pygame main loop
    pygame_loop()
