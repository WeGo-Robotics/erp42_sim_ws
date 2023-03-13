#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
import os


# Define IMU class
class IMU:
    # IMU class constructor
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("imu_node")
        # Create a subscriber that listens to messages on "/imu" topic and sets the callback function to imu_CB.
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_CB)

    # Callback function called when receiving messages from subscriber
    def imu_CB(self, msg):
        # Create a message object
        self.imu_msg = Imu()
        # Store the received msg in traffic_msg
        self.imu_msg = msg

        os.system("clear")  # Clear the terminal
        print(f"------------------------------")
        print(f"angular_velocity :")
        print(f"    x : {self.imu_msg.angular_velocity.x}")  # x
        print(f"    y : {self.imu_msg.angular_velocity.y}")  # y
        print(f"    z : {self.imu_msg.angular_velocity.z}")  # z
        print(f"angular_velocity :")
        print(f"    x : {self.imu_msg.angular_velocity.x}")
        print(f"    y : {self.imu_msg.angular_velocity.y}")
        print(f"    z : {self.imu_msg.angular_velocity.z}")
        print(f"linear_acceleration :")
        print(f"    x : {self.imu_msg.linear_acceleration.x}")
        print(f"    y : {self.imu_msg.linear_acceleration.y}")
        print(f"    z : {self.imu_msg.linear_acceleration.z}")
        print(f"------------------------------")


# When the main script is executed, run the following code.
if __name__ == "__main__":
    # Create IMU class object
    imu = IMU()
    rospy.spin()
