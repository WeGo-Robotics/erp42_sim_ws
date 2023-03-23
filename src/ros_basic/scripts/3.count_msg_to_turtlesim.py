#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from random import *
import os


class Count_msg_to_turtlesim:
    def __init__(self):
        rospy.init_node("sub_and_pub_node")
        self.sub = rospy.Subscriber("count", Int32, callback=self.count_CB)
        self.pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=1)

    def count_CB(self, msg):
        twist_msg = Twist()
        twist_msg.linear.x = (msg.data % 10) / 10 * randint(-1, 1)
        twist_msg.angular.z = (msg.data % 10) / 10 * randint(-1, 1)
        os.system("clear")
        print(f"------------------------------")
        print(f"msg : ", msg)
        print(f"Twist msg : ", twist_msg)
        print(f"------------------------------")


if __name__ == "__main__":
    try:
        count_msg_to_turtlesim = Count_msg_to_turtlesim()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
