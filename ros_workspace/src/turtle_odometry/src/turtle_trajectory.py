#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose as TurtlePose
from turtlesim.srv import TeleportAbsolute, SetPen
from std_srvs.srv import Empty
import numpy as np
import threading

class TurtleTrajectory():
    def __init__(self, name) -> None:
        self.name = name
        self.pub_velocity = rospy.Publisher(
            f"/{self.name}/cmd_vel", Twist, queue_size=1
        )
        self.velocity = Twist()
        self.sub_gt_pose = rospy.Subscriber(
            f"/{self.name}/pose", TurtlePose, self.callback_gt_pose
        )

        # setup test parameters
        rospy.wait_for_service(f"/{self.name}/teleport_absolute")
        rospy.wait_for_service(f"/{self.name}/set_pen")
        rospy.wait_for_service(f"/clear")
        self.srv_teleport_absolute = rospy.ServiceProxy(f"/{self.name}/teleport_absolute", TeleportAbsolute)
        self.srv_set_pen = rospy.ServiceProxy(f"/{self.name}/set_pen", SetPen)
        self.srv_clear = rospy.ServiceProxy(f"/clear", Empty)
        self.srv_set_pen(r=255, g=255, b=255, width=5, off=False)
        self.srv_teleport_absolute(x=1, y=1, theta=0)
        self.srv_clear()

    # setup a command publisher in the background to avoid ROS timing issues
        self.command_event = threading.Event()
        self.command_thread = threading.Thread(target=self.fixed_rate_commands)
        self.command_thread.start()

    def stop_commands(self):
        self.command_event.set()
        self.command_thread.join()
        print("stopped: command publisher")

    def fixed_rate_commands(self):
        '''[thread method] Constantly publish commands in the background
        '''
        r = rospy.Rate(50)
        while not self.command_event.is_set() and not rospy.is_shutdown():
            try:
                self.pub_velocity.publish(self.velocity)
                r.sleep()
            except rospy.ROSInterruptException:
                pass

    def callback_gt_pose(self, msg):
        self.gt_pose = msg

    def closed_loop_square(self, speed=3., segment_length=10.):
        # the turtlesim area is 11 by 11 meters
        # (0, 0) is bottom left / x horizontal to the right / y vertical to the top

        try:
            r = rospy.Rate(70)  # match ground truth pose check rate with its publish rate

            # right
            self.velocity = Twist(linear=Point(x=speed))
            while self.gt_pose.x < segment_length:
                r.sleep()
            # spot turn left
            self.velocity = Twist(angular=Point(z=speed/3))
            while self.gt_pose.theta < np.pi / 2 and self.gt_pose.theta >= 0:
                r.sleep()

            # up
            self.velocity = Twist(linear=Point(x=speed))
            while self.gt_pose.y < segment_length:
                r.sleep()
            # spot turn left
            self.velocity = Twist(angular=Point(z=speed/3))
            while self.gt_pose.theta < np.pi and self.gt_pose.theta >= 0:
                r.sleep()

            # left
            self.velocity = Twist(linear=Point(x=speed))
            while self.gt_pose.x > 1:
                r.sleep()
            # spot turn left
            self.velocity = Twist(angular=Point(z=speed/3))
            while self.gt_pose.theta < -np.pi / 2 and self.gt_pose.theta <= 0:
                r.sleep()

            # down
            self.velocity = Twist(linear=Point(x=speed))
            while self.gt_pose.y > 1:
                r.sleep()
            self.velocity = Twist()  # stop
            rospy.sleep(0.1)

        except (KeyboardInterrupt, rospy.ROSInterruptException):
            self.stop_commands()




def main():
    rospy.init_node("turtle_trajectory")
    try:
        tj = TurtleTrajectory("turtle1")
        rospy.sleep(5)
        print("starting turtle trajectory")
        tj.closed_loop_square(speed=5., segment_length=10.)
        tj.stop_commands()  # to exit script
        # rospy.spin()      # to keep script running
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
