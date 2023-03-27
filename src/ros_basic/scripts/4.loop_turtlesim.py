#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from random import *
import os


class Loop_turtlesim:
    def __init__(self):
        rospy.init_node("loop_turtlesim_node")
        self.sub = rospy.Subscriber("/turtle1/pose", Pose, callback=self.pose_CB)
        self.pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=1)
        self.rate = rospy.Rate(10)

    def pose_CB(self, msg):
        pose_msg = msg
        twist_msg = Twist()
        twist_msg.linear.x = pose_msg.x * randint(-1, 1)
        twist_msg.angular.z = pose_msg.y * randint(-1, 1)
        os.system("clear")
        print(f"------------------------------")
        print(f"Pose msg : ", msg)
        print(f"------------------------------")
        print(f"------------------------------")
        print(f"Twist msg : ", pose_msg)
        print(f"------------------------------")
        self.rate.sleep()


if __name__ == "__main__":
    try:
        loop_turtlesim = Loop_turtlesim()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
