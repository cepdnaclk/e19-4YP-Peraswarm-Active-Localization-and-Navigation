#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle

class RobotMapVisualizer:
    def __init__(self):
        rospy.init_node('robot_map_visualizer')
        
        # Setup figure
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.ax.set_title('Robot Positions on Global Map')
        self.ax.set_xlabel('X (meters)')
        self.ax.set_ylabel('Y (meters)')
        self.ax.grid(True)
        self.ax.axis('equal')
        
        # Robot data storage
        self.robot_positions = {f'tb3_{i}': {'x': 0, 'y': 0, 'theta': 0} for i in range(6)}
        self.robot_artists = {f'tb3_{i}': None for i in range(6)}
        self.global_map = None
        self.map_info = None
        
        # Color scheme for robots
        self.robot_colors = [
            'red', 'blue', 'green', 
            'purple', 'orange', 'cyan'
        ]
        
        # Setup subscribers
        for i in range(6):
            rospy.Subscriber(f'/tb3_{i}/odom', Odometry, 
                           self.robot_odom_callback, 
                           callback_args=f'tb3_{i}')
        
        rospy.Subscriber('/global_map', OccupancyGrid, self.map_callback)
        
        # Initialize visualization elements
        self.map_image = None
        self.robot_labels = []
        
        # Animation setup
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=500)
        
        rospy.loginfo("Robot position visualizer initialized")

    def robot_odom_callback(self, msg, robot_name):
        try:
            # Get position
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            
            # Get orientation (yaw)
            orientation = msg.pose.pose.orientation
            _, _, theta = euler_from_quaternion([
                orientation.x,
                orientation.y, 
                orientation.z,
                orientation.w
            ])
            
            # Update position
            self.robot_positions[robot_name] = {
                'x': x,
                'y': y,
                'theta': theta
            }
            
        except Exception as e:
            rospy.logwarn(f"Error processing {robot_name} odom: {str(e)}")

    def map_callback(self, msg):
        try:
            # Store map data
            self.map_info = msg.info
            self.global_map = np.array(msg.data, dtype=np.int8).reshape(
                (msg.info.height, msg.info.width))
            
            # Calculate map bounds
            self.map_origin_x = msg.info.origin.position.x
            self.map_origin_y = msg.info.origin.position.y
            self.map_width = msg.info.width * msg.info.resolution
            self.map_height = msg.info.height * msg.info.resolution
            
        except Exception as e:
            rospy.logwarn(f"Error processing global map: {str(e)}")

    def update_plot(self, frame):
        try:
            self.ax.clear()
            
            # Draw map if available
            if self.global_map is not None and self.map_info is not None:
                # Create extent in meters
                extent = [
                    self.map_origin_x,
                    self.map_origin_x + self.map_width,
                    self.map_origin_y,
                    self.map_origin_y + self.map_height
                ]
                
                # Display map (flip y-axis for proper orientation)
                self.ax.imshow(
                    np.flipud(self.global_map), 
                    cmap='binary',
                    extent=extent,
                    vmin=-1,
                    vmax=1,
                    origin='lower'
                )
            
            # Draw each robot
            for i, (robot_name, pos) in enumerate(self.robot_positions.items()):
                x, y, theta = pos['x'], pos['y'], pos['theta']
                
                # Draw robot position
                robot_circle = plt.Circle(
                    (x, y), 
                    0.1,  # Robot radius
                    color=self.robot_colors[i],
                    alpha=0.8
                )
                self.ax.add_patch(robot_circle)
                
                # Draw orientation arrow
                dx = 0.2 * np.cos(theta)
                dy = 0.2 * np.sin(theta)
                self.ax.arrow(
                    x, y, dx, dy, 
                    head_width=0.05, 
                    head_length=0.1, 
                    fc=self.robot_colors[i], 
                    ec=self.robot_colors[i]
                )
                
                # Add robot label
                self.ax.text(
                    x, y + 0.15, 
                    robot_name, 
                    color=self.robot_colors[i],
                    ha='center',
                    bbox=dict(facecolor='white', alpha=0.7, edgecolor='none')
                )
            
            # Set plot limits based on map or robot positions
            if self.global_map is not None:
                self.ax.set_xlim([
                    self.map_origin_x - 1,
                    self.map_origin_x + self.map_width + 1
                ])
                self.ax.set_ylim([
                    self.map_origin_y - 1,
                    self.map_origin_y + self.map_height + 1
                ])
            else:
                # Fallback limits if no map
                self.ax.set_xlim([-5, 5])
                self.ax.set_ylim([-5, 5])
            
            self.ax.grid(True)
            self.ax.set_title('Robot Positions on Global Map')
            self.ax.set_xlabel('X (meters)')
            self.ax.set_ylabel('Y (meters)')
            
        except Exception as e:
            rospy.logerr(f"Error updating plot: {str(e)}")

    def run(self):
        try:
            plt.tight_layout()
            plt.show(block=True)
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down visualizer")
        except Exception as e:
            rospy.logerr(f"Visualization error: {str(e)}")

if __name__ == "__main__":
    visualizer = RobotMapVisualizer()
    visualizer.run()