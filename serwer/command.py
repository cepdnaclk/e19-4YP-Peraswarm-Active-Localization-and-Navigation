a#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class BoundedMultiRobotCommander:
    def __init__(self, robot_names):
        rospy.init_node('bounded_multi_robot_cmd_vel', anonymous=True)
        self.robot_names = robot_names
        self.publishers = {}
        self.positions = {name: [0.0, 0.0] for name in robot_names}  # x, y
        self.velocities = {name: 0.2 for name in robot_names}  # initial forward velocity

        for name in self.robot_names:
            cmd_topic = f'/{name}/cmd_vel'
            odom_topic = f'/{name}/odom'
            self.publishers[name] = rospy.Publisher(cmd_topic, Twist, queue_size=10)
            rospy.Subscriber(odom_topic, Odometry, self.odom_callback, callback_args=name)
            rospy.loginfo(f"Set up for {name}: cmd_vel → {cmd_topic}, odom → {odom_topic}")

        rospy.sleep(1.0)

    def odom_callback(self, msg, robot_name):
        self.positions[robot_name] = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ]

    def move_all(self):
        for name in self.robot_names:
            x, y = self.positions[name]
            v = self.velocities[name]

            # Check for boundary conditions
            if x >= 4.0 or x <= -4.0 or y >= 4.0 or y <= -4.0:
                self.velocities[name] *= -1  # Reverse direction
                rospy.logwarn(f"{name} hit boundary at ({x:.2f}, {y:.2f}) → Reversing velocity to {self.velocities[name]:.2f}")

          # Publish command
            cmd = Twist()
            cmd.linear.x = self.velocities[name]
            cmd.angular.z = 0.0
            self.publishers[name].publish(cmd)
            rospy.loginfo(f"{name} at (x={x:.2f}, y={y:.2f}) → v={self.velocities[name]:.2f}")

if __name__ == '__main__':
    try:
        robots = [f'tb3_{i}' for i in range(6)]
        commander = BoundedMultiRobotCommander(robots)
        rate = rospy.Rate(5)  # 5 Hz update

        while not rospy.is_shutdown():
            commander.move_all()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
