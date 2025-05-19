#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
import os

def clear_terminal():
    os.system('cls' if os.name == 'nt' else 'clear')

def print_map(matrix):
    for row in matrix:
        print(" ".join(f"{int(cell):2d}" for cell in row))

def map_callback(msg):
    width = msg.info.width
    height = msg.info.height
    data = np.array(msg.data).reshape((height, width))

    clear_terminal()
    print("Live Map View (from /tb3_0/local_map)\n")
    print_map(data)

def main():
    rospy.init_node("map_terminal_viewer")
    rospy.Subscriber("/tb3_0/local_map", OccupancyGrid, map_callback)
    rospy.loginfo("Map Terminal Viewer started. Waiting for map...")
    rospy.spin()

if __name__ == "__main__":
    main()
