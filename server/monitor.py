# monitor.py
import requests
import pygame
import time

SERVER_MAP_URL = "http://localhost:5000/global_map"

MAP_SIZE = 100
CELL_SIZE = 8  # Adjust cell size to fit big map on screen
WINDOW_SIZE = MAP_SIZE * CELL_SIZE

pygame.init()
screen = pygame.display.set_mode((WINDOW_SIZE, WINDOW_SIZE))
pygame.display.set_caption("Global Map Monitor")
clock = pygame.time.Clock()

def fetch_global_map():
    try:
        response = requests.get(SERVER_MAP_URL, timeout=2)
        if response.status_code == 200:
            return response.json()["global_map"]
    except Exception as e:
        print(f"[Monitor] Failed to fetch map: {e}")
    return None

def draw_global_map(global_map):
    screen.fill((255, 255, 255))  # white background
    for y in range(MAP_SIZE):
        for x in range(MAP_SIZE):
            cell = global_map[y][x]
            rect = pygame.Rect(x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE)
            if cell == 'B':
                pygame.draw.rect(screen, (0, 128, 255), rect)  # bot presence
            elif cell == 'O':
                pygame.draw.rect(screen, (0, 255, 0), rect)    # obstacle
            elif cell == 'X':
                pygame.draw.rect(screen, (0, 0, 0), rect)      # wall
            else:
                pygame.draw.rect(screen, (220, 220, 220), rect, 1)  # empty space

    pygame.display.flip()

def monitor_loop():
    running = True
    while running:
        clock.tick(2)  # Refresh 2 times per second
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        global_map = fetch_global_map()
        if global_map:
            draw_global_map(global_map)

    pygame.quit()
    exit()

if __name__ == "__main__":
    monitor_loop()
