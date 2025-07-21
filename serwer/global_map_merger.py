#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker
import threading

class MapMerger:
    def __init__(self):
        rospy.init_node('map_merger_with_pose_tracing')

        self.global_resolution = 0.1
        self.global_size = 120  # 12m x 12m map
        self.global_map = np.full((self.global_size, self.global_size), -1, dtype=np.int8)

        self.robot_poses = {}
        self.robot_maps = {}

        self.lock = threading.Lock()

        self.num_robots = 6
        self.robot_names = [f'tb3_{i}' for i in range(self.num_robots)]

        for name in self.robot_names:
            self.robot_poses[name] = {'x': 0, 'y': 0, 'theta': 0}
            self.robot_maps[name] = None

            rospy.Subscriber(f"/{name}/odom", Odometry, self.odom_callback, callback_args=name)
            rospy.Subscriber(f"/{name}/local_map", OccupancyGrid, self.map_callback, callback_args=name)

        self.global_map_pub = rospy.Publisher('/global_map', OccupancyGrid, queue_size=10)
        self.marker_pub = rospy.Publisher('/robot_debug_markers', Marker, queue_size=10)

        rospy.loginfo("Map merger with pose tracing initialized")

    def odom_callback(self, msg, robot_name):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        with self.lock:
            self.robot_poses[robot_name] = {
                'x': pos.x,
                'y': pos.y,
                'theta': yaw
            }
        rospy.loginfo(f"📍[{robot_name}] Pose updated → x: {pos.x:.2f}, y: {pos.y:.2f}, θ: {np.degrees(yaw):.2f}°")

    def map_callback(self, msg, robot_name):
        with self.lock:
            height, width = msg.info.height, msg.info.width
            resolution = 0.1  # Assuming a fixed resolution for simplicity
            pose = self.robot_poses[robot_name]

            map_data = np.array(msg.data, dtype=np.int8).reshape((height, width))
            corrected_origin = Point(
                x=pose['x'] - 3.0,  # Map spans 6x6 meters, centered on robot
                y=pose['y'] - 3.0,
                z=0
            )
            self.robot_maps[robot_name] = {
                'data': map_data,
                'resolution': resolution,
                'origin': corrected_origin
            }

            rospy.loginfo(f" [{robot_name}] Local map received → Origin: ({corrected_origin.x:.2f}, {corrected_origin.y:.2f}), Size: {width}x{height}")
            self.merge_map(robot_name)

    def merge_map(self, robot_name):
        local = self.robot_maps[robot_name]
        pose = self.robot_poses[robot_name]

        if local is None:
            return

        map_data = local['data']
        res = local['resolution']
        origin = local['origin']
        robot_theta = pose['theta']

        height, width = map_data.shape

        global_origin_x = -self.global_size * self.global_resolution / 2
        global_origin_y = -self.global_size * self.global_resolution / 2

        for y in range(height):
            for x in range(width):
                value = map_data[y, x]
                if value == -1:
                    continue

                local_x = x * res
                local_y = y * res

                rotated_x = local_x * np.cos(robot_theta) - local_y * np.sin(robot_theta)
                rotated_y = local_x * np.sin(robot_theta) + local_y * np.cos(robot_theta)

                global_x = origin.x + rotated_x
                global_y = origin.y + rotated_y

                global_ix = int((global_x - global_origin_x) / self.global_resolution)
                global_iy = int((global_y - global_origin_y) / self.global_resolution)

                if 0 <= global_ix < self.global_size and 0 <= global_iy < self.global_size:
                    if self.global_map[global_iy, global_ix] == -1 or value > self.global_map[global_iy, global_ix]:
                        self.global_map[global_iy, global_ix] = value

        rospy.loginfo(f"[{robot_name}] Merged local map into global map")
        self.publish_robot_marker(robot_name, origin.x + 3, origin.y + 3)  # Center of the robot
        self.publish_global_map()

    def publish_global_map(self):
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "global_map"

        msg.info.resolution = self.global_resolution
        msg.info.width = self.global_size
        msg.info.height = self.global_size

        msg.info.origin = Pose(
            position=Point(
                x=-self.global_size * self.global_resolution / 2,
                y=-self.global_size * self.global_resolution / 2,
                z=0
            ),
            orientation=Quaternion(x=0, y=0, z=0, w=1)
        )

        msg.data = self.global_map.flatten().tolist()
        self.global_map_pub.publish(msg)
        rospy.loginfo("Published updated global map")

    def publish_robot_marker(self, robot_name, x, y):
        marker = Marker()
        marker.header.frame_id = "global_map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "robots"
        marker.id = hash(robot_name) % 100
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.1

        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.marker_pub.publish(marker)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    merger = MapMerger()
    merger.run()
