#!/usr/bin/env python3
import rospy
import socket
import json
import numpy as np
import sys
import time
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return super().default(obj)

class ConnectionNode:
    def __init__(self):
        rospy.init_node('bot_client', anonymous=True)
        self.namespace = rospy.get_namespace().strip('/')
        self.robot_name = self.namespace
        self.robot_id = self.robot_name.split('_')[-1]

        self.HOST = '127.0.0.1'
        self.PORT = 9005

        self.obs_data = {'scan': [], 'odom': []}
        self.tb_yaw = 0.0

        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('imu', Imu, self.imu_callback)
        rospy.Subscriber('local_map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('odom', Odometry, self.odom_callback)

        rospy.loginfo(f"[RL CLIENT] Started in namespace: /{self.robot_name}")

    def normalize_angle(self, angle):
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    def action_to_angle(self, action):
        direction_map = {
            (1, 0): 0,
            (0, 1): np.pi / 2,
            (-1, 0): np.pi,
            (0, -1): -np.pi / 2
        }
        return direction_map.get(tuple(action), 0.0)

    def rotate_and_move(self, dtheta):
        MAX_ANGULAR_SPEED = 0.5
        LINEAR_SPEED = 0.1
        FORWARD_DURATION = 1.0

        dtheta = self.normalize_angle(dtheta)
        rotate_time = abs(dtheta) / MAX_ANGULAR_SPEED
        direction = 1 if dtheta > 0 else -1

        twist = Twist()
        twist.angular.z = direction * MAX_ANGULAR_SPEED
        self.cmd_pub.publish(twist)
        time.sleep(rotate_time)

        self.cmd_pub.publish(Twist())
        time.sleep(0.1)

        twist = Twist()
        twist.linear.x = LINEAR_SPEED
        self.cmd_pub.publish(twist)
        time.sleep(FORWARD_DURATION)

        self.cmd_pub.publish(Twist())

    def imu_callback(self, msg):
        pass  # Optional

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x * 10 % 1000
        y = msg.pose.pose.position.y * 10 % 1000
        quat = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        self.tb_yaw = yaw
        self.obs_data['odom'] = np.array([x, y, 30, 30], dtype=np.float32)

    def map_callback(self, msg):
        grid = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        local = grid.astype(np.float32).reshape(1, msg.info.height, msg.info.width, 1)
        self.obs_data['scan'] = local

    def send_recv_action(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.connect((self.HOST, self.PORT))
                    s.sendall(json.dumps(self.obs_data, cls=NumpyEncoder).encode())
                    response = json.loads(s.recv(1024).decode())
                    action = response['action']

                    target_angle = self.action_to_angle(action)
                    dtheta = self.normalize_angle(target_angle - self.tb_yaw)
                    rospy.loginfo(f"[{self.robot_name}] Action: {action}, Yaw: {self.tb_yaw:.2f}, Δθ: {dtheta:.2f}")

                    self.rotate_and_move(dtheta)

            except Exception as e:
                rospy.logwarn(f"[{self.robot_name}] Connection error: {e}")
            rate.sleep()

    def run(self):
        rospy.loginfo(f"[{self.robot_name}] Starting control loop")
        self.send_recv_action()


if __name__ == "__main__":
    try:
        node = ConnectionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
