#!/usr/bin/env python3
import rospy
import numpy as np
import json
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion

# === Constants and Map ===
MAP_SIZE = 100
MAP_RESOLUTION = 0.05  # 5cm
grid_map = np.full((MAP_SIZE, MAP_SIZE), -1, dtype=np.int8)

# EKF State
state = np.array([MAP_SIZE // 2, MAP_SIZE // 2, 0.0], dtype=np.float64)  # [x, y, theta]
P = np.eye(3) * 0.1
Q = np.eye(3) * 0.01

monitor_clients = []  # Placeholder if needed for external use

# === EKF Functions ===

def ekf_predict(state, P, imu_data):
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
    P = F @ P @ F.T + Q
    return state, P

def ekf_update(state, P, lidar_data):
    for distance, angle in lidar_data:
        angle_world = state[2] + angle
        obs_x = state[0] + distance * np.cos(angle_world)
        obs_y = state[1] + distance * np.sin(angle_world)

        map_x, map_y = int(obs_x), int(obs_y)
        if 0 <= map_x < MAP_SIZE and 0 <= map_y < MAP_SIZE:
            grid_map[map_y, map_x] = 1

        # Free space
        steps = int(distance)
        for step in range(steps):
            fx = state[0] + step * np.cos(angle_world)
            fy = state[1] + step * np.sin(angle_world)
            ix, iy = int(fx), int(fy)
            if 0 <= ix < MAP_SIZE and 0 <= iy < MAP_SIZE:
                if grid_map[iy, ix] == -1:
                    grid_map[iy, ix] = 0
    return state, P
def publish_local_map(map_pub):
    msg = OccupancyGrid()
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"

    msg.info.resolution = MAP_RESOLUTION
    msg.info.width = MAP_SIZE
    msg.info.height = MAP_SIZE
    msg.info.origin = Pose(
        position=Point(x=-MAP_SIZE * MAP_RESOLUTION / 2, y=-MAP_SIZE * MAP_RESOLUTION / 2, z=0),
        orientation=Quaternion(x=0, y=0, z=0, w=1)
    )

    msg.data = grid_map.flatten().tolist()
    map_pub.publish(msg)
    rospy.loginfo("[Map] Published updated local occupancy grid.")
def broadcast_map():
    map_data = grid_map.tolist()
    message = json.dumps({"map": map_data}).encode()
    for client in monitor_clients:
        try:
            client.sendall(message)
        except:
            monitor_clients.remove(client)

# === ROS Callbacks ===

def imu_callback(msg):
    global state, P
    # Simple velocity model (approx)
    linear_acc = msg.linear_acceleration
    angular_vel = msg.angular_velocity

    # Convert acceleration to dx/dy assuming dt ≈ 1
    dx = linear_acc.x
    dy = linear_acc.y
    dtheta = angular_vel.z

    rospy.loginfo("[IMU] Predicting state")
    state, P = ekf_predict(state, P, (dx, dy, dtheta))

def scan_callback(msg):
    global state, P
    lidar_data = []

    angle = msg.angle_min
    for r in msg.ranges:
        if msg.range_min < r < msg.range_max:
            lidar_data.append((r, angle))
        angle += msg.angle_increment

    rospy.loginfo(f"[LIDAR] Updating map with {len(lidar_data)} rays")
    state, P = ekf_update(state, P, lidar_data)

def map_callback(msg):
    rospy.loginfo(f"[Map] Received local map: {msg.info.width}x{msg.info.height}")

# === Main ===

def main():
    '''rospy.init_node("robot_mapping_node")

    rospy.Subscriber("/tb3_0/imu", Imu, imu_callback)
    rospy.Subscriber("/tb3_0/scan", LaserScan, scan_callback)
    rospy.Subscriber("/tb3_0/local_map", OccupancyGrid, map_callback)

    rospy.loginfo("[Main] Subscribed to IMU, scan, and local map.")
    rospy.spin()'''
    global map_pub
    rospy.init_node("robot_mapping_node")

    rospy.Subscriber("/tb3_0/imu", Imu, imu_callback)
    rospy.Subscriber("/tb3_0/scan", LaserScan, scan_callback)
    rospy.Subscriber("/tb3_0/local_map", OccupancyGrid, map_callback)

    map_pub = rospy.Publisher("/tb3_0/local_map", OccupancyGrid, queue_size=10)

    rospy.loginfo("[Main] Subscribed to IMU, scan, and local map. Ready to publish.")

    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        publish_local_map(map_pub)
        rate.sleep()

if __name__ == "__main__":
    main()
