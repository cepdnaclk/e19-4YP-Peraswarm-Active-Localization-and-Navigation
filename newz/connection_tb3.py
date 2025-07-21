#!/usr/bin/env python3
import rospy
import socket
import json
import numpy as np
import math
import time
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

obs_data = {'scan': [], 'odom': []}
tb_yaw = 0.0  # Global yaw variable

# Optional: Handle IMU if needed
def imu_callback(msg):
    pass

# Action to angle
def action_to_angle(action, degrees=False):
    direction_map = {
        (1, 0): 0,
        (0, 1): math.pi / 2,
        (-1, 0): math.pi,
        (0, -1): -math.pi / 2
    }
    action_tuple = tuple(action)
    angle_rad = direction_map.get(action_tuple, None)
    if angle_rad is None:
        raise ValueError(f"Invalid action {action}. Must be one of [1,0], [0,1], [-1,0], [0,-1]")
    return math.degrees(angle_rad) if degrees else angle_rad

# Normalize angle between [-pi, pi]
def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

# Rotate by angle and move forward
def rotate_and_move(dtheta, cmd_pub):
    MAX_ANGULAR_SPEED = 0.5
    LINEAR_SPEED = 0.1
    FORWARD_DURATION = 1.0

    dtheta = normalize_angle(dtheta)
    rotate_time = abs(dtheta) / MAX_ANGULAR_SPEED
    angular_direction = 1 if dtheta > 0 else -1

    # ROTATE
    twist = Twist()
    twist.angular.z = angular_direction * MAX_ANGULAR_SPEED
    cmd_pub.publish(twist)
    time.sleep(rotate_time)

    cmd_pub.publish(Twist())
    time.sleep(0.1)

    # MOVE FORWARD
    twist = Twist()
    twist.linear.x = LINEAR_SPEED
    cmd_pub.publish(twist)
    time.sleep(FORWARD_DURATION)

    cmd_pub.publish(Twist())

# Extract 20x20 local map (placeholder)
def get_local_map(global_map, robot_pose, size=20):
    return global_map[20:40, 20:40]  # just a slice for example

# OccupancyGrid callback
def map_callback(msg):
    width, height = msg.info.width, msg.info.height
    data = msg.data
    grid = [data[y * width:(y + 1) * width] for y in range(height)]
    global_map = np.array(grid)
    local_map = get_local_map(global_map, [0, 0], size=20)
    state_grid = local_map.astype(np.float32).reshape(1, 20, 20, 1)
    obs_data['scan'] = state_grid
    rospy.loginfo_throttle(5, f"[OccupancyGrid] Map size: {width}x{height}")

# Odometry callback
def odom_callback(msg):
    global tb_yaw
    x, y =msg.pose.pose.position.x*10, msg.pose.pose.position.y*10
    x = x%1000
    y = y%1000
    orientation_q = msg.pose.pose.orientation
    _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
    tb_yaw = yaw  # update global yaw
    obs_data['odom'] = np.array([x, y, 30,30], dtype=np.float32)
    rospy.loginfo_throttle(1, f"[odom_callback] Robot at x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")

# Socket communication and movement
def send_recv_action():
    pub = rospy.Publisher('/tb3_0/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect(('localhost', 9002))
                s.sendall(json.dumps(obs_data, cls=NumpyEncoder).encode())
                data = s.recv(1024)
                action = json.loads(data.decode())
                action_vec = action['action']

                target_angle = action_to_angle(action_vec)
                dtheta = normalize_angle(target_angle - tb_yaw)

                rospy.loginfo(f"[RL CLIENT] Yaw: {tb_yaw:.2f}, Target: {target_angle:.2f}, dtheta: {dtheta:.2f}")

                rotate_and_move(dtheta, pub)
                rospy.loginfo(f"[RL CLIENT] Executed action: {action_vec}")

        except Exception as e:
            rospy.logwarn(f"[RL CLIENT] Connection/data error: {e}")

        rate.sleep()

# Encode numpy to JSON
class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)

# Main entry
if __name__ == '__main__':
    rospy.init_node('rl_bridge_node')
    rospy.Subscriber("/tb3_0/imu", Imu, imu_callback)
    rospy.Subscriber("/tb3_0/odom", Odometry, odom_callback)
    rospy.Subscriber("/tb3_0/local_map", OccupancyGrid, map_callback)
    rospy.sleep(1.0)
    rospy.loginfo("[RL CLIENT] Node started. Waiting for data...")
    send_recv_action()