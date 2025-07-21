#!/usr/bin/env python3
import rospy
import numpy as np
import json
import math
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion

class RobotMappingNode:
    def __init__(self):
        # Get namespace and robot name
        self.namespace = rospy.get_namespace()
        self.robot_name = self.namespace.strip('/')
        
        # Map configuration
        self.MAP_SIZE = 60
        self.MAP_RESOLUTION = 0.1  # 5cm per cell
        self.grid_map = np.full((self.MAP_SIZE, self.MAP_SIZE), -1, dtype=np.int8)
        
        # EKF State (x, y, theta)
        self.state = np.array([self.MAP_SIZE // 2, self.MAP_SIZE // 2, 0.0], dtype=np.float64)
        self.P = np.eye(3) * 0.1  # Covariance matrix
        self.Q = np.eye(3) * 0.01  # Process noise
        
        # Initialize ROS node
        rospy.init_node('mapping_node', anonymous=True)
        
        # Setup subscribers and publishers
        rospy.Subscriber('imu', Imu, self.imu_callback)
        rospy.Subscriber('scan', LaserScan, self.scan_callback)
        rospy.Subscriber('local_map', OccupancyGrid, self.map_callback)
        self.map_pub = rospy.Publisher('local_map', OccupancyGrid, queue_size=10)
        
        rospy.loginfo(f"[{self.robot_name}] SLAM node initialized")

    # ===== EKF Functions =====
    def ekf_predict(self, state, P, imu_data):
        dx, dy, dtheta = imu_data
        theta = state[2]

        # Predict next state
        state[0] += dx * np.cos(theta) - dy * np.sin(theta)
        state[1] += dx * np.sin(theta) + dy * np.cos(theta)
        state[2] += dtheta

        # Jacobian
        F = np.array([
            [1, 0, -dx * np.sin(theta) - dy * np.cos(theta)],
            [0, 1, dx * np.cos(theta) - dy * np.sin(theta)],
            [0, 0, 1]
        ])

        # Predict covariance
        P = F @ P @ F.T + self.Q
        return state, P

    def ekf_update(self, state, P, lidar_data):
        for distance, angle in lidar_data:
            angle_world = state[2] + angle
            obs_x = state[0] + distance * np.cos(angle_world)
            obs_y = state[1] + distance * np.sin(angle_world)

            map_x, map_y = int(obs_x), int(obs_y)
            if 0 <= map_x < self.MAP_SIZE and 0 <= map_y < self.MAP_SIZE:
                self.grid_map[map_y, map_x] = 1

            # Free space
            steps = int(distance)
            for step in range(steps):
                fx = state[0] + step * np.cos(angle_world)
                fy = state[1] + step * np.sin(angle_world)
                ix, iy = int(fx), int(fy)
                if 0 <= ix < self.MAP_SIZE and 0 <= iy < self.MAP_SIZE:
                    if self.grid_map[iy, ix] == -1:
                        self.grid_map[iy, ix] = 0
        return state, P

    # ===== Utility Functions =====
    def polar_to_cartesian(self, r, angle_rad):
        x = r * math.cos(angle_rad)
        y = r * math.sin(angle_rad)
        return x, y

    def world_to_grid(self, x, y):
        gx = int(self.state[0] + x / self.MAP_RESOLUTION)
        gy = int(self.state[1] + y / self.MAP_RESOLUTION)
        return gx, gy

    def bresenham_line(self, x0, y0, x1, y1):
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = -1 if x0 > x1 else 1
        sy = -1 if y0 > y1 else 1
        
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                points.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                points.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        points.append((x1, y1))
        return points

    # ===== ROS Callbacks =====
    def imu_callback(self, msg):
        # Simple velocity model (approx)
        linear_acc = msg.linear_acceleration
        angular_vel = msg.angular_velocity

        # Convert acceleration to dx/dy assuming dt ≈ 1
        dx = linear_acc.x
        dy = linear_acc.y
        dtheta = angular_vel.z

        rospy.loginfo(f"[{self.robot_name}] IMU Update - dx: {dx:.4f}, dy: {dy:.4f}, dtheta: {dtheta:.4f}")
        self.state, self.P = self.ekf_predict(self.state, self.P, (dx, dy, dtheta))

    def scan_callback(self, msg):
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        range_min = msg.range_min
        range_max = msg.range_max

        robot_x, robot_y = int(self.state[0]), int(self.state[1])
        valid_ranges = 0

        for i, r in enumerate(ranges):
            angle = angle_min + i * angle_increment

            if r == float('inf'):
                r = range_max
                is_obstacle = False
            elif r < range_min or r > range_max:
                continue
            else:
                is_obstacle = True
                valid_ranges += 1

            obs_x, obs_y = self.polar_to_cartesian(r, angle + self.state[2])
            grid_x, grid_y = self.world_to_grid(obs_x, obs_y)

            if 0 <= grid_x < self.MAP_SIZE and 0 <= grid_y < self.MAP_SIZE:
                line_points = self.bresenham_line(robot_x, robot_y, grid_x, grid_y)
                for px, py in line_points[:-1] if is_obstacle else line_points:
                    if 0 <= px < self.MAP_SIZE and 0 <= py < self.MAP_SIZE:
                        if self.grid_map[py, px] == -1:
                            self.grid_map[py, px] = 0

                if is_obstacle:
                    self.grid_map[grid_y, grid_x] = 1

        rospy.loginfo(f"[{self.robot_name}] Processed {valid_ranges} valid LIDAR readings")

    def map_callback(self, msg):
        rospy.loginfo(f"[{self.robot_name}] Received map update")

    def publish_local_map(self):
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = f"{self.robot_name}/map"

        msg.info.resolution = self.MAP_RESOLUTION
        msg.info.width = self.MAP_SIZE
        msg.info.height = self.MAP_SIZE
        msg.info.origin = Pose(
            position=Point(x=-self.MAP_SIZE * self.MAP_RESOLUTION / 2, 
                          y=-self.MAP_SIZE * self.MAP_RESOLUTION / 2, 
                          z=0),
            orientation=Quaternion(x=0, y=0, z=0, w=1)
        )

        msg.data = self.grid_map.flatten().tolist()
        self.map_pub.publish(msg)
        rospy.loginfo(f"[{self.robot_name}] Published local map")

    def run(self):
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            self.publish_local_map()
            rate.sleep()

if __name__ == "__main__":
    try:
        node = RobotMappingNode()
        node.run()
    except rospy.ROSInterruptException:
        pass