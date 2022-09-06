#!/usr/bin/env python3
import rospy
from turtlesim.msg import Pose as TurtlePose
import random
import numpy as np

class TurtleOdom():
    def __init__(self, name):
        self.name = name
        self.cb_throttle = rospy.Time.now()

        # initialize odometry estimate
        self.prev_pose = rospy.wait_for_message(
            f'/{self.name}/pose',
            TurtlePose,
            timeout=None
        )
        self.odometry = self.prev_pose

        # setup odometry publisher
        self.pub_odometry = rospy.Publisher(
            f'{self.name}/odometry',
            TurtlePose,
            queue_size=2
        )
        # for demo purposes: mock odometry
        # subscribe to pose ground truth and register a callback that will add noise and calculate the odometry
        self.sub_pose = rospy.Subscriber(
            f'/{self.name}/pose',
            TurtlePose,
            callback=self.cb_odometry
        )

        rospy.loginfo(f'TurtleOdometry initialized with coordinates: x={self.prev_pose.x} / y={self.prev_pose.y}')

    def cb_odometry(self, odom):
        ### assumes that the turtle only goes forward and spot turn
        time = rospy.Time.now()
        if time - self.cb_throttle > rospy.Duration(0.1):
            self.cb_throttle = time
        else:
            return

        # first, mock an odometry input from the ground truth (+ noise)
        delta_theta = odom.theta - self.prev_pose.theta
        delta_x = odom.x - self.prev_pose.x
        delta_y = odom.y - self.prev_pose.y
        delta_forward = np.sqrt(delta_x**2 + delta_y**2)
        self.prev_pose = odom

        noise_factor_theta = 0.1
        noise_factor_forward = 0.5
        noise_theta = noise_factor_theta * random.uniform(-1, 1)
        noise_forward = noise_factor_forward * random.uniform(-1, 1)

        odom_theta = delta_theta * (1 + noise_theta)
        odom_forward = delta_forward * (1 + noise_forward)


        # Second, update the odometry (position estimate in the map frame)
        self.odometry.theta += odom_theta
        self.odometry.x     += odom_forward * np.cos(self.odometry.theta)
        self.odometry.y     += odom_forward * np.sin(self.odometry.theta)

        # Finaly, publish the estimated odometry
        self.pub_odometry.publish(self.odometry)


def main():
    rospy.init_node("turtle_odometry")
    try:
        rospy.sleep(3)
        to = TurtleOdom("turtle1")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
