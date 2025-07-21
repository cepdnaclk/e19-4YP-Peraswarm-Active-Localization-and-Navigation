import socket
import json
import numpy as np
import os
import time

HOST = 'localhost'
PORT = 9001

def clear_terminal():
    os.system('cls' if os.name == 'nt' else 'clear')

def print_map(matrix):
    for row in matrix:
        print(" ".join(f"{int(cell):2d}" for cell in row))

def run_terminal_monitor():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client:
        client.connect((HOST, PORT))
        print("Connected to map monitor server (Terminal Mode)")

        buffer = b""
        while True:
            try:
                data = client.recv(4096)
                if not data:
                    break
                buffer += data

                try:
                    message = json.loads(buffer.decode())
                    buffer = b""  # Reset buffer if parsing successful
                    if 'map' in message:
                        matrix = np.array(message['map'])

                        clear_terminal()
                        print("Live 50x50 Map View (Terminal)\n")
                        print_map(matrix)
                        time.sleep(0.5)  # Refresh rate
                except json.JSONDecodeError:
                    continue  # Wait for full JSON
            except Exception as e:
                print("Error receiving map:", e)
                break

if __name__ == "__main__":
    run_terminal_monitor()
