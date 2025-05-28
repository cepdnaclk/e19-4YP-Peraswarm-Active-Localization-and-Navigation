#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion
from message_filters import Subscriber, ApproximateTimeSynchronizer
import threading

class MapMerger:
    def __init__(self):
        rospy.init_node('map_merger')

        self.global_map_size = 120
        self.global_resolution = 0.1
        self.global_map = np.full((self.global_map_size, self.global_map_size), -1, dtype=np.int8)

        self.robot_maps = {}
        self.robot_poses = {}
        self.robot_velocities = {}

        for i in range(6):
            robot_name = f"tb3_{i}"
            self.robot_maps[robot_name] = None
            self.robot_poses[robot_name] = (0.0, 0.0, 0.0)
            self.robot_velocities[robot_name] = (0.0, 0.0, 0.0)

        self.setup_subscribers()
        self.merged_map_pub = rospy.Publisher('/global_map', OccupancyGrid, queue_size=10)
        self.lock = threading.Lock()

        rospy.loginfo("Map merger node initialized")

    def setup_subscribers(self):
        for i in range(6):
            robot_name = f"tb3_{i}"
            map_sub = Subscriber(f'/{robot_name}/local_map', OccupancyGrid)
            odom_sub = Subscriber(f'/{robot_name}/odom', PoseWithCovarianceStamped)
            ats = ApproximateTimeSynchronizer([map_sub, odom_sub], queue_size=10, slop=0.1)
            ats.registerCallback(self.synced_callback, robot_name)

            rospy.Subscriber(f'/{robot_name}/cmd_vel', Twist, self.velocity_callback, callback_args=robot_name)

    def synced_callback(self, map_msg, odom_msg, robot_name):
        with self.lock:
            position = odom_msg.pose.pose.position
            orientation = odom_msg.pose.pose.orientation
            _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
            self.robot_poses[robot_name] = (position.x, position.y, yaw)

            map_data = np.array(map_msg.data, dtype=np.int8).reshape((map_msg.info.height, map_msg.info.width))
            self.robot_maps[robot_name] = {
                'data': map_data,
                'resolution': map_msg.info.resolution,
                'origin': map_msg.info.origin,
                'timestamp': map_msg.header.stamp
            }

            self.process_new_map(robot_name)

    def velocity_callback(self, msg, robot_name):
        with self.lock:
            self.robot_velocities[robot_name] = (msg.linear.x, msg.linear.y, msg.angular.z)

    def process_new_map(self, robot_name):
        if robot_name not in self.robot_maps or self.robot_maps[robot_name] is None:
            return

        local_map = self.robot_maps[robot_name]
        map_data = local_map['data']
        resolution = local_map['resolution']
        local_origin = local_map['origin']

        robot_x, robot_y, robot_theta = self.robot_poses[robot_name]

        origin_x_global = robot_x + local_origin.position.x * np.cos(robot_theta) - local_origin.position.y * np.sin(robot_theta)
        origin_y_global = robot_y + local_origin.position.x * np.sin(robot_theta) + local_origin.position.y * np.cos(robot_theta)

        height, width = map_data.shape
        for y in range(height):
            for x in range(width):
                value = map_data[y, x]
                if value == -1:
                    continue

                local_x = x * resolution
                local_y = y * resolution

                global_x = origin_x_global + local_x * np.cos(robot_theta) - local_y * np.sin(robot_theta)
                global_y = origin_y_global + local_x * np.sin(robot_theta) + local_y * np.cos(robot_theta)

                global_ix = int((global_x + (self.global_map_size * self.global_resolution / 2)) / self.global_resolution)
                global_iy = int((global_y + (self.global_map_size * self.global_resolution / 2)) / self.global_resolution)

                if 0 <= global_ix < self.global_map_size and 0 <= global_iy < self.global_map_size:
                    if self.global_map[global_iy, global_ix] == -1 or value > self.global_map[global_iy, global_ix]:
                        self.global_map[global_iy, global_ix] = value

        self.publish_global_map()

    def publish_global_map(self):
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "global_map"

        msg.info.resolution = self.global_resolution
        msg.info.width = self.global_map_size
        msg.info.height = self.global_map_size

        msg.info.origin = Pose(
            position=Point(
                x=-self.global_map_size * self.global_resolution / 2,
                y=-self.global_map_size * self.global_resolution / 2,
                z=0
            ),
            orientation=Quaternion(x=0, y=0, z=0, w=1)
        )

        msg.data = self.global_map.flatten().tolist()
        self.merged_map_pub.publish(msg)
        rospy.loginfo("Published updated global map")

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            with self.lock:
                self.publish_global_map()
            rate.sleep()

if __name__ == "__main__":
    merger = MapMerger()
    merger.run()
