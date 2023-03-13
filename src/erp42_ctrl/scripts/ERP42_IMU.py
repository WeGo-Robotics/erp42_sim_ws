#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
import os


class IMU:
    def __init__(self):
        rospy.init_node("imu_node")
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_CB)

    # imu 정보 콜백 함수
    def imu_CB(self, msg):
        self.imu_msg = Imu()
        self.imu_msg = msg

        os.system("clear")
        print(f"------------------------------")
        print(f"angular_velocity :")
        print(f"    x : {self.imu_msg.angular_velocity.x}")
        print(f"    y : {self.imu_msg.angular_velocity.y}")
        print(f"    z : {self.imu_msg.angular_velocity.z}")
        print(f"angular_velocity :")
        print(f"    x : {self.imu_msg.angular_velocity.x}")
        print(f"    y : {self.imu_msg.angular_velocity.y}")
        print(f"    z : {self.imu_msg.angular_velocity.z}")
        print(f"linear_acceleration :")
        print(f"    x : {self.imu_msg.linear_acceleration.x}")
        print(f"    y : {self.imu_msg.linear_acceleration.y}")
        print(f"    z : {self.imu_msg.linear_acceleration.z}")
        print(f"------------------------------")


if __name__ == "__main__":
    imu = IMU()
    rospy.spin()
