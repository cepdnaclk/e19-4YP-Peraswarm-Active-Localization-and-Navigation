import pygame
import random
import threading
import time

# Configuration
MAP_SIZE = 20        # 20x20 grid
CELL_SIZE = 30       # Each cell is 30x30 pixels
WINDOW_SIZE = MAP_SIZE * CELL_SIZE
BOT_COUNT = 10       # Number of bots

# Pygame Initialization
pygame.init()
screen = pygame.display.set_mode((WINDOW_SIZE, WINDOW_SIZE))
pygame.display.set_caption(f"Swarm Simulation")
clock = pygame.time.Clock()

# Bot Colors
colors = [(random.randint(50,255), random.randint(50,255), random.randint(50,255)) for _ in range(BOT_COUNT)]

# Bot State
bots = []
for bot_id in range(BOT_COUNT):
    bot = {
        "id": bot_id,
        "position": [random.randint(0, MAP_SIZE-1), random.randint(0, MAP_SIZE-1)],
        "color": colors[bot_id],
    }
    bots.append(bot)

def get_random_direction():
    return random.choice(['UP', 'DOWN', 'LEFT', 'RIGHT'])

def move_bot(bot):
    direction = get_random_direction()

    if direction == 'UP' and bot["position"][1] > 0:
        bot["position"][1] -= 1
    elif direction == 'DOWN' and bot["position"][1] < MAP_SIZE - 1:
        bot["position"][1] += 1
    elif direction == 'LEFT' and bot["position"][0] > 0:
        bot["position"][0] -= 1
    elif direction == 'RIGHT' and bot["position"][0] < MAP_SIZE - 1:
        bot["position"][0] += 1

def draw_map():
    screen.fill((255, 255, 255))  # White background
    # Draw grid
    for y in range(MAP_SIZE):
        for x in range(MAP_SIZE):
            rect = pygame.Rect(x*CELL_SIZE, y*CELL_SIZE, CELL_SIZE, CELL_SIZE)
            pygame.draw.rect(screen, (220, 220, 220), rect, 1)  # Light grid

    # Draw bots
    for bot in bots:
        x, y = bot["position"]
        rect = pygame.Rect(x*CELL_SIZE, y*CELL_SIZE, CELL_SIZE, CELL_SIZE)
        pygame.draw.rect(screen, bot["color"], rect)

    pygame.display.flip()

def bot_loop():
    while True:
        for bot in bots:
            move_bot(bot)
        time.sleep(0.5)  # All bots move every 0.5 seconds

def pygame_loop():
    running = True
    while running:
        clock.tick(30)  # 30 FPS refresh rate
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        draw_map()

    pygame.quit()
    exit()

if __name__ == "__main__":
    # Run bot movement in a separate thread
    threading.Thread(target=bot_loop, daemon=True).start()

    # Run the Pygame main loop
    pygame_loop()
