#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np

class LocalMapPublisher:
    def __init__(self):
        rospy.init_node("local_map_publisher")
        
        # Publisher for this robot's local map
        self.map_pub = rospy.Publisher("/tb3_0/local_map", OccupancyGrid, queue_size=10)
        
        # Subscriber to receive the published map (for debug or testing)
        rospy.Subscriber("/tb3_0/local_map", OccupancyGrid, self.map_callback)
        
        self.map_size = 100
        self.resolution = 0.05  # 5 cm per cell

        self.origin = Pose(
            position=Point(x=-2.5, y=-2.5, z=0.0),  # Map centered around (0,0)
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )

        self.map_data = np.full((self.map_size, self.map_size), -1, dtype=np.int8)  # Unknown space

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"

        msg.info.resolution = self.resolution
        msg.info.width = self.map_size
        msg.info.height = self.map_size
        msg.info.origin = self.origin

        msg.data = self.map_data.flatten().tolist()
        self.map_pub.publish(msg)
        rospy.loginfo("[Map Publisher] Published local map")

    def map_callback(self, msg):
        rospy.loginfo(f"[LocalMapReceiver] Received updated local map {msg.info.width}x{msg.info.height}")
        local_map_np = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        # Optional: print or analyze the received map data
        # print(local_map_np)

    def update_and_publish_loop(self):
        rate = rospy.Rate(1)  # Publish at 1 Hz
        while not rospy.is_shutdown():
            # Example: mark center cell as free (0)
            self.map_data[self.map_size // 2, self.map_size // 2] = 0
            self.publish_map()
            rate.sleep()

if __name__ == "__main__":
    publisher = LocalMapPublisher()
    publisher.update_and_publish_loop()

