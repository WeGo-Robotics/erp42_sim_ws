#!/usr/bin/env python3

import rospy
from morai_msgs.msg import GPSMessage
import os


class GPS:
    def __init__(self):
        rospy.init_node("gps_node")
        self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.gps_CB)

    # gps 정보 콜백 함수
    def gps_CB(self, msg):
        self.gps_msg = GPSMessage()
        self.gps_msg = msg

        os.system("clear")
        print(f"------------------------------")
        print(f"latitude : {self.gps_msg.latitude}")
        print(f"longitude : {self.gps_msg.longitude}")
        print(f"altitude : {self.gps_msg.altitude}")
        print(f"------------------------------")


if __name__ == "__main__":
    gps = GPS()
    rospy.spin()
