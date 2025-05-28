#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class SimplePosePlotter:
    def __init__(self):
        rospy.init_node('simple_pose_plotter')

        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.ax.set_title('XY Pose of Multiple Robots')
        self.ax.set_xlim(-6, 6)
        self.ax.set_ylim(-6, 6)
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.grid(True)

        self.robot_names = [f"tb3_{i}" for i in range(6)]
        self.colors = ['red', 'blue', 'green', 'purple', 'orange', 'cyan']

        self.robot_positions = {name: (0.0, 0.0) for name in self.robot_names}

        for name in self.robot_names:
            rospy.Subscriber(f"/{name}/odom", Odometry, self.odom_cb, callback_args=name)
            rospy.loginfo(f"Subscribed to /{name}/odom")

        self.ani = FuncAnimation(self.fig, self.update_plot, interval=200)

    def odom_cb(self, msg, name):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.robot_positions[name] = (x, y)

    def update_plot(self, frame):
        self.ax.clear()
        self.ax.set_title("XY Pose of Multiple Robots")
        self.ax.set_xlim(-6, 6)
        self.ax.set_ylim(-6, 6)
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.grid(True)

        for i, name in enumerate(self.robot_names):
            x, y = self.robot_positions[name]
            self.ax.plot(x, y, 'o', color=self.colors[i], markersize=10)
            self.ax.text(x, y + 0.2, name, color=self.colors[i], ha='center')

    def run(self):
        try:
            plt.tight_layout()
            plt.show()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down pose plotter")

if __name__ == '__main__':
    plotter = SimplePosePlotter()
    plotter.run()
