#!/usr/bin/env python3

import rospy
from morai_msgs.msg import GetTrafficLightStatus
import os


class Get_traffic:
    def __init__(self):
        rospy.init_node("get_traffic_node")
        self.get_traffic_sub = rospy.Subscriber(
            "/GetTrafficLightStatus", GetTrafficLightStatus, self.traffic_CB
        )

    # traffic 정보 콜백 함수
    def traffic_CB(self, msg):
        self.traffic_msg = GetTrafficLightStatus()
        self.traffic_msg = msg
        os.system("clear")
        print(f"------------------------------")
        print(f"trafficLightIndex : {self.traffic_msg.trafficLightIndex}")
        print(f"trafficLightType : {self.traffic_msg.trafficLightType}")
        print(f"trafficLightStatus : {self.traffic_msg.trafficLightStatus}")
        print(f"------------------------------")


if __name__ == "__main__":
    try:
        get_traffic = Get_traffic()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
