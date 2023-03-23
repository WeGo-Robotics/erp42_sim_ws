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
        imu_msg = Imu()
        # Store the received msg in imu_msg
        imu_msg = msg
        # Clear the terminal
        os.system("clear")
        print(f"------------------------------")

        # Print angular_velocity
        print(f"angular_velocity :")
        print(f"    x : {imu_msg.angular_velocity.x}")
        print(f"    y : {imu_msg.angular_velocity.y}")
        print(f"    z : {imu_msg.angular_velocity.z}")

        # Print linear_acceleration
        print(f"linear_acceleration :")
        print(f"    x : {imu_msg.linear_acceleration.x}")
        print(f"    y : {imu_msg.linear_acceleration.y}")
        print(f"    z : {imu_msg.linear_acceleration.z}")
        print(f"------------------------------")


# When the main script is executed, run the following code.
if __name__ == "__main__":
    try:
        # Create IMU class object
        imu = IMU()

        # Continuously call callback function in an infinite loop
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
