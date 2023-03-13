#!/usr/bin/env python3

import rospy
from morai_msgs.msg import EgoVehicleStatus
import os


class Control:
    def __init__(self):
        rospy.init_node("ERP42_status_node")
        self.obj_sub = rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_CB)

    def status_CB(self, msg):
        self.status_msg = EgoVehicleStatus()
        self.status_msg = msg
        os.system("clear")
        print(f"------------------------------")
        print(f"acceleration :")
        print(f"    x : {self.status_msg.acceleration.x}")
        print(f"    y : {self.status_msg.acceleration.y}")
        print(f"    z : {self.status_msg.acceleration.z}")
        print(f"position :")
        print(f"    x : {self.status_msg.position.x}")
        print(f"    y : {self.status_msg.position.y}")
        print(f"    z : {self.status_msg.position.z}")
        print(f"velocity :")
        print(f"    x : {self.status_msg.velocity.x}")
        print(f"    y : {self.status_msg.velocity.y}")
        print(f"    z : {self.status_msg.velocity.z}")
        print(f"heading : {self.status_msg.heading}")
        print(f"accel : {self.status_msg.accel}")
        print(f"brake : {self.status_msg.brake}")
        print(f"wheel_angle : {self.status_msg.wheel_angle}")
        print(f"------------------------------")


if __name__ == "__main__":
    try:
        control = Control()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
