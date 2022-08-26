#!/usr/bin/env python
import sys
import unittest
from std_msgs.msg import String
import rospy

PKG = 'l3_testing'
NAME = 'turtle'
TIMEOUT = 5


class TestTurtle(unittest.TestCase):

    ## runs before each test_* method
    # def setUp(self):
    #     rospy.init_node("testurtle")

    def test_demo(self):
        self.assertEqual(1, 1)

    def test_turtle(self):
        rospy.init_node("testurtle", anonymous=True)
        self.result = rospy.wait_for_message(
            "/turtle1/result", String, timeout=TIMEOUT
        )
        self.assertEqual(
            self.result.data,
            "Turtle nearing right edge",
            f"got unexpected result: {self.result}"
        )


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestTurtle, sys.argv)
