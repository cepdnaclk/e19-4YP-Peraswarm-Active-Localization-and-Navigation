#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Ellipse

class MultiRobotTracker:
    def __init__(self):
        rospy.init_node('multi_robot_tracker')
        
        # Setup 12x12 meter plot
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.ax.set_title('6-Robot Tracker with EKF (12x12m)')
        self.ax.set_xlabel('X (meters)')
        self.ax.set_ylabel('Y (meters)')
        self.ax.set_xlim(-6, 6)
        self.ax.set_ylim(-6, 6)
        self.ax.grid(True)
        
        # Robot colors and properties
        self.colors = ['red', 'blue', 'green', 'purple', 'orange', 'cyan']
        self.robot_names = [f'tb3_{i}' for i in range(6)]
        
        # EKF states and covariance for each robot
        self.ekf_states = {name: np.zeros(3) for name in self.robot_names}  # [x, y, theta]
        self.ekf_cov = {name: np.eye(3)*0.1 for name in self.robot_names}   # Covariance matrix
        
        # Process noise and measurement noise
        self.Q = np.diag([0.1, 0.1, 0.05])**2  # Process noise
        self.R = np.diag([0.5, 0.5, 0.1])**2   # Measurement noise
        
        # Visualization elements
        self.robot_plots = {name: None for name in self.robot_names}
        self.cov_ellipses = {name: None for name in self.robot_names}
        
        # Setup subscribers for each robot
        for name in self.robot_names:
            rospy.Subscriber(f'/{name}/odom', Odometry, self.odom_cb, name)
            rospy.Subscriber(f'/{name}/imu', Imu, self.imu_cb, name)
            rospy.loginfo(f"Subscribed to /{name}/odom and /{name}/imu")
        
        # Animation setup
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=200)
        
        rospy.loginfo("Multi-robot EKF tracker initialized")

    def ekf_predict(self, name, u, dt):
        """EKF prediction step"""
        x, y, theta = self.ekf_states[name]
        v, w = u  # linear and angular velocity
        
        # State prediction
        new_theta = theta + w*dt
        new_x = x + v*np.cos(theta)*dt
        new_y = y + v*np.sin(theta)*dt
        
        # Jacobian of motion model
        F = np.array([
            [1, 0, -v*np.sin(theta)*dt],
            [0, 1, v*np.cos(theta)*dt],
            [0, 0, 1]
        ])
        
        # Update state and covariance
        self.ekf_states[name] = np.array([new_x, new_y, new_theta])
        self.ekf_cov[name] = F @ self.ekf_cov[name] @ F.T + self.Q

    def ekf_update(self, name, z):
        """EKF update step"""
        H = np.eye(3)  # Measurement matrix
        y = z - H @ self.ekf_states[name]  # Innovation
        S = H @ self.ekf_cov[name] @ H.T + self.R
        K = self.ekf_cov[name] @ H.T @ np.linalg.inv(S)  # Kalman gain
        
        # Update state and covariance
        self.ekf_states[name] += K @ y
        self.ekf_cov[name] = (np.eye(3) - K @ H) @ self.ekf_cov[name]

    def odom_cb(self, msg, name):
        """Odometry callback with logging"""
        try:
            # Get pose and velocities
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            z = msg.pose.pose.position.z
            v = msg.twist.twist.linear.x
            w = msg.twist.twist.angular.z
            
            # Log the odometry data
            rospy.loginfo(f"{name} Odometry - Position: x={x:.2f}m, y={y:.2f}m, z={z:.2f}m | Velocity: v={v:.2f}m/s, w={w:.2f}rad/s")
            
            # Time since last update (crude approximation)
            dt = 0.1  
            
            # EKF prediction step
            self.ekf_predict(name, (v, w), dt)
            
        except Exception as e:
            rospy.logwarn(f"Odom error for {name}: {str(e)}")

    def imu_cb(self, msg, name):
        """IMU callback with logging"""
        try:
            # Get orientation from IMU
            orientation = msg.orientation
            roll, pitch, yaw = euler_from_quaternion([
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w
            ])
            
            # Log IMU data
            rospy.loginfo(f"{name} IMU - Orientation: roll={roll:.2f}rad, pitch={pitch:.2f}rad, yaw={yaw:.2f}rad")
            
            # Get position from current state
            x, y, _ = self.ekf_states[name]
            
            # EKF update step
            self.ekf_update(name, np.array([x, y, yaw]))
            
        except Exception as e:
            rospy.logwarn(f"IMU error for {name}: {str(e)}")

    def update_plot(self, frame):
        """Update the visualization"""
        self.ax.clear()
        self.ax.set_xlim(-6, 6)
        self.ax.set_ylim(-6, 6)
        self.ax.grid(True)
        self.ax.set_title('6-Robot Tracker with EKF (12x12m)')
        
        # Draw each robot with covariance ellipse
        for i, name in enumerate(self.robot_names):
            x, y, theta = self.ekf_states[name]
            cov = self.ekf_cov[name]
            
            # Log current EKF state
            rospy.loginfo(f"{name} EKF State - x={x:.2f}m, y={y:.2f}m, θ={theta:.2f}rad")
            
            # Draw robot
            self.ax.plot(x, y, 'o', color=self.colors[i], markersize=10)
            
            # Draw orientation arrow
            dx = 0.3 * np.cos(theta)
            dy = 0.3 * np.sin(theta)
            self.ax.arrow(x, y, dx, dy, 
                         head_width=0.1, 
                         head_length=0.15, 
                         fc=self.colors[i], 
                         ec=self.colors[i])
            
            # Draw covariance ellipse (position only)
            if cov[0,0] > 0 and cov[1,1] > 0:
                lambda_, v = np.linalg.eig(cov[:2,:2])
                angle = np.degrees(np.arctan2(v[1,0], v[0,0]))
                width = 2 * np.sqrt(lambda_[0]) * 2  # 2-sigma
                height = 2 * np.sqrt(lambda_[1]) * 2
                
                ellipse = Ellipse(xy=(x, y),
                                width=width,
                                height=height,
                                angle=angle,
                                color=self.colors[i],
                                alpha=0.2)
                self.ax.add_patch(ellipse)
            
            # Add label
            self.ax.text(x, y+0.3, name, color=self.colors[i], ha='center')

    def run(self):
        try:
            plt.tight_layout()
            plt.show(block=True)
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down tracker")
        except Exception as e:
            rospy.logerr(f"Visualization error: {str(e)}")

if __name__ == "__main__":
    tracker = MultiRobotTracker()
    tracker.run()