# bot.py
import requests
import time
import random
import threading
import pygame

SERVER_URL = "http://localhost:5000/upload"

MAP_SIZE = 10
CELL_SIZE = 50
WINDOW_SIZE = MAP_SIZE * CELL_SIZE

bot_id = random.randint(1000, 9999)
position = [random.randint(10, 90), random.randint(10, 90)]  # Not too close to edge

local_map = [['.' for _ in range(MAP_SIZE)] for _ in range(MAP_SIZE)]

pygame.init()
screen = pygame.display.set_mode((WINDOW_SIZE, WINDOW_SIZE))
pygame.display.set_caption(f"Bot {bot_id}")
clock = pygame.time.Clock()

def get_imu_reading():
    return random.choice(['UP', 'DOWN', 'LEFT', 'RIGHT'])

def get_lidar_reading():
    return random.uniform(0.5, 5.0)

def update_local_map(x, y):
    for i in range(MAP_SIZE):
        for j in range(MAP_SIZE):
            if local_map[i][j] == 'B':
                local_map[i][j] = '.'
    local_map[y][x] = 'B'

def draw_map():
    screen.fill((255, 255, 255))
    for y in range(MAP_SIZE):
        for x in range(MAP_SIZE):
            rect = pygame.Rect(x*CELL_SIZE, y*CELL_SIZE, CELL_SIZE, CELL_SIZE)
            if local_map[y][x] == 'B':
                pygame.draw.rect(screen, (0, 128, 255), rect)
            else:
                pygame.draw.rect(screen, (220, 220, 220), rect, 1)
    pygame.display.flip()

def move_self():
    direction = get_imu_reading()
    if direction == 'UP' and position[1] > 0:
        position[1] -= 1
    elif direction == 'DOWN' and position[1] < 99:
        position[1] += 1
    elif direction == 'LEFT' and position[0] > 0:
        position[0] -= 1
    elif direction == 'RIGHT' and position[0] < 99:
        position[0] += 1

def send_to_server():
    try:
        payload = {
            "bot_id": bot_id,
            "position": {"x": position[0], "y": position[1]},
            "local_map": local_map
        }
        response = requests.post(SERVER_URL, json=payload, timeout=2)
        if response.status_code == 200:
            data = response.json()
            update_local_map_from_server(data['local_map'])
            print(f"[Bot {bot_id}] Map updated from server.")
    except Exception as e:
        print(f"[Bot {bot_id}] Failed to connect to server: {e}")

def update_local_map_from_server(new_map):
    for y in range(MAP_SIZE):
        for x in range(MAP_SIZE):
            local_map[y][x] = new_map[y][x]

def bot_loop():
    while True:
        move_self()
        update_local_map(5, 5)  # Bot always in center of its map
        send_to_server()
        time.sleep(1)

def pygame_loop():
    running = True
    while running:
        clock.tick(10)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        draw_map()
    pygame.quit()
    exit()

if __name__ == "__main__":
    threading.Thread(target=bot_loop, daemon=True).start()
    pygame_loop()
