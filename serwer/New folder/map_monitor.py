#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid
from matplotlib.animation import FuncAnimation

class GlobalMapVisualizer:
    def __init__(self):
        rospy.init_node('global_map_visualizer')
        
        # Setup matplotlib figure
        self.fig, self.ax = plt.subplots(figsize=(12, 12))
        self.img = self.ax.imshow(np.zeros((120, 120)), cmap='binary', vmin=-1, vmax=1)
        plt.colorbar(self.img, label='Occupancy Probability')
        self.ax.set_title('Global Occupancy Map')
        self.ax.set_xlabel('X (cells)')
        self.ax.set_ylabel('Y (cells)')
        
        # Subscribe to global map
        rospy.Subscriber('/global_map', OccupancyGrid, self.map_callback)
        
        # Animation setup
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=1000)
        
        rospy.loginfo("Global map visualizer initialized")

    def map_callback(self, msg):
        # Convert map data to numpy array
        self.map_data = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        self.resolution = msg.info.resolution
        self.width = msg.info.width
        self.height = msg.info.height
        
        rospy.loginfo(f"Received new map: {self.width}x{self.height} @ {self.resolution}m/cell")

    def update_plot(self, frame):
        if hasattr(self, 'map_data'):
            # Update the image data
            self.img.set_array(self.map_data)
            
            # Invert Y axis to match ROS convention (origin at bottom)
            self.ax.invert_yaxis()
            
            # Update plot labels with current dimensions
            self.ax.set_title(f'Global Occupancy Map ({self.width}x{self.height} @ {self.resolution}m/cell)')
            
            # Redraw the figure
            self.fig.canvas.draw()

    def run(self):
        plt.show(block=True)
        rospy.spin()

if __name__ == "__main__":
    try:
        visualizer = GlobalMapVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        pass