#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import random

class SimpleAvoidance:
    def __init__(self):
        rospy.init_node('simple_avoidance')
        self.cmd_pub = rospy.Publisher('/tb3_0/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/tb3_0/scan', LaserScan, self.scan_callback)
        self.twist = Twist()
        self.run()

    def scan_callback(self, msg):
        # Front 30° section
        ranges = np.array(msg.ranges)
        front_ranges = np.concatenate((ranges[:15], ranges[-15:]))
        min_dist = np.min(front_ranges)

        if min_dist < 0.5:
            self.twist.linear.x = 0.0
            self.twist.angular.z = random.choice([-1.0, 1.0])  # Turn randomly
        else:
            self.twist.linear.x = 0.1
            self.twist.angular.z = 0.0

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.cmd_pub.publish(self.twist)
            rate.sleep()

if __name__ == "__main__":
    SimpleAvoidance()
