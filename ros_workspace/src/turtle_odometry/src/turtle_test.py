#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose as TurtlePose


class TestListener():
    def __init__(self):
        self.turtle_pose_sub = rospy.Subscriber(
            "/turtle1/pose", TurtlePose, self.gt_callback
        )
        self.velocity_publisher = rospy.Publisher(
            "/turtle1/cmd_vel", Twist, queue_size=10)
        self.result_publisher = rospy.Publisher(
            "/turtle1/result", String, queue_size=10, latch=True)
        self.gt_pose = None
        self.twist = Twist()

    def gt_callback(self, msg):
        vel_msg = Twist()
        vel_msg.linear.x = 1.0
        self.velocity_publisher.publish(vel_msg)
        self.gt_pose = msg
        if self.gt_pose.x > 8:
            result = String("Turtle nearing right edge")
            self.result_publisher.publish(result)


def main():
    rospy.init_node("test_listener")
    TestListener()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
