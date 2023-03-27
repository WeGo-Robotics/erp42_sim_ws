#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from random import *
from math import *
import os

pose_msg = Pose()


class Avoid_wall:
    def __init__(self):
        rospy.init_node("avoid_node")
        self.sub = rospy.Subscriber("/turtle1/pose", Pose, callback=self.pose_CB)
        self.pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=1)
        self.rate = rospy.Rate(10)
        self.angle_to_wall = 0
        self.reflection_angle = 0
        self.warning_flag = False

    def pose_CB(self, msg):
        pose_msg = msg
        twist_msg = Twist()
        turtle_angle = pose_msg.theta
        twist_msg.linear.x = random() * 2

        # If the turtle is within the specified range of x and y values
        if 1 <= pose_msg.x <= 10 and 1 <= pose_msg.y <= 10:
            twist_msg.angular.z = random() * 4 - 2
            self.warning_flag = False
        else:
            if self.warning_flag == False:
                # Calculate the angle between the turtle and the wall
                self.angle_to_wall = atan2(pose_msg.y - 5, pose_msg.x - 5)
                # Calculate the angle of reflection
                self.reflection_angle = self.angle_to_wall + pi
                if self.reflection_angle > pi:
                    self.reflection_angle = self.reflection_angle - pi * 2
                self.warning_flag = True

            # Set the turtle's angular velocity to rotate towards the reflection angle
            twist_msg.angular.z = self.reflection_angle - pose_msg.theta

        os.system("clear")
        print(f"------------------------------")
        print(f"Pose_msg : \n", msg)
        print(f"------------------------------")
        print("")
        print(f"------------------------------")
        print(f"Twist_msg : \n", twist_msg)
        print(f"------------------------------")
        print("")
        print(f"------------------------------")
        print(f"turtle_angle : ", turtle_angle * 180 / pi)
        print(f"self.warning_flag : ", self.warning_flag)
        print(f"angle_to_wall : ", self.angle_to_wall * 180 / pi)
        print(f"reflection_degree : ", self.reflection_angle * 180 / pi)
        print(f"twist_msg.angular.z : ", twist_msg.angular.z * 180 / pi)
        print(f"------------------------------")

        # Publish the velocity command to the turtle
        self.pub.publish(twist_msg)
        self.rate.sleep()


if __name__ == "__main__":
    try:
        avoid_wall = Avoid_wall()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
