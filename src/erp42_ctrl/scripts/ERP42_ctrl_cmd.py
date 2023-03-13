#!/usr/bin/env python3

import rospy
from morai_msgs.msg import CtrlCmd
from math import *
from time import *


class ERP42_ctrl_cmd:
    def __init__(self):
        rospy.init_node("erp42_ctrl_cmd_node")
        self.start_time = rospy.get_time()
        self.erp42_ctrl_pub = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=3)
        self.steering_bound = 28.2

    def main(self):
        self.ctrl_cmd_msg = CtrlCmd()
        self.second_time = rospy.get_time()

        speed = 15  # 0 ~ 23.8 -> 0 ~ 1
        speed = max(0, min(speed, 23.8))
        steering_deg = 10
        steering_deg = max(-self.steering_bound, min(steering_deg, self.steering_bound))
        steering_rad = steering_deg * pi / 180
        steering_deg = -steering_deg
        self.ctrl_cmd_msg.accel = speed
        self.ctrl_cmd_msg.steering = steering_rad

        if self.second_time - self.start_time >= 3:
            self.start_time = self.second_time
            self.erp42_ctrl_pub.publish(self.ctrl_cmd_msg)


if __name__ == "__main__":
    try:
        erp42_ctrl_cmd = ERP42_ctrl_cmd()

        while not rospy.is_shutdown():
            erp42_ctrl_cmd.main()

    except rospy.ROSInterruptException:
        pass
