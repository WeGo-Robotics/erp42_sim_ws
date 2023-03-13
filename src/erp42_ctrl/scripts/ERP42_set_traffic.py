#!/usr/bin/env python3

import rospy
from morai_msgs.msg import SetTrafficLight
from math import *
from time import *


class Set_traffic:
    def __init__(self):
        rospy.init_node("set_traffic_node")
        self.start_time = rospy.get_time()
        self.traffic_pub = rospy.Publisher(
            "/SetTrafficLight", SetTrafficLight, queue_size=3
        )
        self.traffic_info = [
            [58.50, 1180.41, "C119BS010001"],  ##신호등 정보(global_x, global_y, index)
            [85.61, 1227.88, "C119BS010021"],
            [136.58, 1351.98, "C119BS010025"],
            [141.02, 1458.27, "C119BS010028"],
            [139.39, 1596.44, "C119BS010033"],
            [48.71, 1208.02, "C119BS010005"],
            [95.58, 1181.56, "C119BS010047"],
            [104.46, 1161.46, "C119BS010046"],
            [85.29, 1191.77, "C119BS010007"],
            [106.32, 1237.04, "C119BS010022"],
            [75.34, 1250.43, "C119BS010024"],
            [73.62, 1218.01, "C119BS010012"],
            [116.37, 1190.65, "C119BS010040"],
            [153.98, 1371.48, "C119BS010073"],
            [129.84, 1385.08, "C119BS010039"],
            [116.28, 1367.77, "C119BS010074"],
            [75.08, 1473.34, "C119BS010075"],
            [67.10, 1506.66, "C119BS010076"],
            [114.81, 1485.81, "C119BS010079"],
            [159.11, 1496.63, "C119BS010060"],
            [122.24, 1608.26, "C119BS010072"],
            [132.70, 1624.78, "C119BS010034"],
        ]

    def main(self):
        self.set_traffic_msg = SetTrafficLight()

        self.traffic_number = 0

        self.traffic_x = self.traffic_info[self.traffic_number][0]
        self.traffic_y = self.traffic_info[self.traffic_number][1]
        self.traffic_name = self.traffic_info[self.traffic_number][2]

        Red = 1
        Yellow = 4
        Red_Yellow = 5
        Green = 16
        Yellow_Green = 20
        GreenLeft = 32
        Yellow_GreenLeft = 36
        Green_GreenLeft = 48
        default = -1
        signal = Green_GreenLeft

        self.second_time = rospy.get_time()
        if self.second_time - self.start_time >= 1:
            self.start_time = self.second_time
            self.set_traffic_msg.trafficLightIndex = self.traffic_name
            self.set_traffic_msg.trafficLightStatus = signal
            self.traffic_pub.publish(self.set_traffic_msg)


if __name__ == "__main__":
    set_traffic = Set_traffic()
    while not rospy.is_shutdown():
        set_traffic.main()
