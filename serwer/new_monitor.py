import socket
import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

HOST = 'localhost'
PORT = 9001

fig, ax = plt.subplots()
map_img = ax.imshow(np.full((50, 50), -1), cmap='gray', vmin=-1, vmax=1)
plt.title("Live 50x50 Map View (Gray=-1, Black=0, White=1)")

def update_map(frame):
    global map_img
    try:
        data = client.recv(4096)
        if not data:
            return

        message = json.loads(data.decode())
        if 'map' in message:
            matrix = np.array(message['map'])
            map_img.set_data(matrix)
    except Exception as e:
        print("Error receiving map:", e)

    return [map_img]

# Connect to monitor server
client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect((HOST, PORT))
print("Connected to map monitor server")

ani = animation.FuncAnimation(fig, update_map, interval=500, blit=True)
plt.show()
