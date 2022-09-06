#!/usr/bin/env python
import sys
import unittest
from std_srvs.srv import Empty as EmptySrv
from turtlesim.srv import TeleportAbsolute, SetPen
from turtlesim.msg import Pose as TurtlePose
import rospy
import numpy as np
from turtle_trajectory import TurtleTrajectory

PKG = 'turtle_odometry'
NAME = 'turtle1'


class TestTurtle(unittest.TestCase):

    # runs before each test_* method
    def setUp(self):
        ## Arange
        rospy.init_node("testurtle", anonymous=True)
        # setup the turtlesim simulator
        rospy.wait_for_service(f"/{NAME}/teleport_absolute")
        rospy.wait_for_service(f"/{NAME}/set_pen")
        rospy.wait_for_service(f"/clear")
        self.srv_teleport_absolute = rospy.ServiceProxy(f"/{NAME}/teleport_absolute", TeleportAbsolute)
        self.srv_set_pen = rospy.ServiceProxy(f"/{NAME}/set_pen", SetPen)
        self.srv_clear = rospy.ServiceProxy(f"/clear", EmptySrv)
        self.srv_set_pen(r=255, g=255, b=255, width=5, off=False)
        self.srv_teleport_absolute(x=1, y=1, theta=0)
        self.srv_clear()
        # prepare trajectory commands for the test
        self.turtle_trajectory = TurtleTrajectory(NAME)
        rospy.loginfo("setting up turtle trajectory")
        # reset odometry to pose of test setup
        rospy.wait_for_service(f"/{NAME}/odom_reset")
        self.srv_reset_odom = rospy.ServiceProxy(f"/{NAME}/odom_reset", EmptySrv)
        self.srv_reset_odom()

    def test_turtle(self):
        ## Act
        # log some data while not moving
        rospy.sleep(2)
        # start moving
        self.turtle_trajectory.closed_loop_square(speed=5., segment_length=10.)
        self.turtle_trajectory.stop_commands()  # to stop publisher
        # log some data after stopped moving
        rospy.sleep(1)

        ## Assert
        final_pose = rospy.wait_for_message(
            f'/{NAME}/pose',
            TurtlePose,
            timeout=None
        )
        distance_to_start = np.sqrt((final_pose.x - 1)**2 + (final_pose.y - 1)**2)
        rospy.logerr(f"distance to start: {distance_to_start}")
        # turtle moved (= is not at exactly the starting position)
        self.assertNotEqual(
            distance_to_start,
            0,
            msg=f"turtle position at end of test exactly at starting position: did not move?"
        )
        # turtle is back near starting position
        self.assertAlmostEqual(
            distance_to_start,
            0,
            delta=0.5,
            msg=f"turtle position at end of test is more than 0.5m away from starting position: error in trajectory?"
        )


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestTurtle, sys.argv)
