#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid
from matplotlib.animation import FuncAnimation

class MultiMapVisualizer:
    def __init__(self):
        rospy.init_node('multi_map_visualizer')
        
        # Create figure with 7 subplots (6 robots + global)
        self.fig, self.axes = plt.subplots(3, 3, figsize=(15, 15))
        self.fig.suptitle('Multi-Robot Mapping System', fontsize=16)
        
        # Flatten axes array for easier indexing
        self.axes = self.axes.flatten()
        
        # Initialize image plots for each map
        self.imgs = []
        for i in range(7):  # 6 robots + global
            img = self.axes[i].imshow(np.zeros((60, 60)), 
                             cmap='binary',
                             vmin=-1,
                             vmax=1,
                             interpolation='none')
            self.imgs.append(img)
            if i < 6:
                self.axes[i].set_title(f'Robot tb3_{i} Map')
            else:
                self.axes[i].set_title('Global Merged Map')
            plt.colorbar(img, ax=self.axes[i])
        
        # Hide unused subplot (since we have 7 plots in 3x3 grid)
        for i in range(7, 9):
            self.axes[i].axis('off')
        
        # Store map data
        self.robot_maps = {f'tb3_{i}': None for i in range(6)}
        self.global_map = None
        
        # Setup subscribers
        for i in range(6):
            rospy.Subscriber(f'/tb3_{i}/local_map', OccupancyGrid, 
                           self.robot_map_callback, 
                           callback_args=f'tb3_{i}')
        
        rospy.Subscriber('/global_map', OccupancyGrid, self.global_map_callback)
        
        # Animation setup
        self.ani = FuncAnimation(self.fig, self.update_plots, interval=1000)
        
        rospy.loginfo("Multi-map visualizer initialized")

    def robot_map_callback(self, msg, robot_name):
        try:
            # Convert map data to numpy array
            map_data = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
            self.robot_maps[robot_name] = map_data
        except Exception as e:
            rospy.logwarn(f"Error processing {robot_name} map: {str(e)}")

    def global_map_callback(self, msg):
        try:
            # Convert global map data
            self.global_map = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        except Exception as e:
            rospy.logwarn(f"Error processing global map: {str(e)}")

    def update_plots(self, frame):
        try:
            # Update robot maps
            for i in range(6):
                robot_name = f'tb3_{i}'
                if self.robot_maps[robot_name] is not None:
                    self.imgs[i].set_array(self.robot_maps[robot_name])
                    self.axes[i].invert_yaxis()
            
            # Update global map
            if self.global_map is not None:
                self.imgs[6].set_array(self.global_map)
                self.axes[6].invert_yaxis()
            
            # Redraw the figure
            self.fig.canvas.draw_idle()
            
        except Exception as e:
            rospy.logerr(f"Error updating plots: {str(e)}")

    def run(self):
        try:
            plt.tight_layout()
            plt.show(block=True)
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down visualizer")
        except Exception as e:
            rospy.logerr(f"Visualization error: {str(e)}")

if __name__ == "__main__":
    visualizer = MultiMapVisualizer()
    visualizer.run()