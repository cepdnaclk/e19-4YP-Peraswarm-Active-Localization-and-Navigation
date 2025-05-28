import requests
import time
import random

SERVER_URL = "http://localhost:5000/upload"

bot_id = 100

while True:
    sensor_data = {
        "bot_id": bot_id,
        "timestamp": time.time(),
        "value": random.uniform(0, 100)
    }
    try:
        response = requests.post(SERVER_URL, json=sensor_data)
        if response.status_code == 200:
            print(f"[Bot {bot_id}] Successfully sent data: {sensor_data['value']:.2f}")
    except Exception as e:
        print(f"[Bot {bot_id}] Error sending data: {e}")
    
    time.sleep(1)  # Send data every 1 second
