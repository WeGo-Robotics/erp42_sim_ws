#!/usr/bin/env python3

import rospy
from morai_msgs.msg import CtrlCmd
from math import *
from time import *


# Define ERP42_ctrl_cmd class
class ERP42_ctrl_cmd:
    # ERP_ctrl_cmd class constructor
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("erp42_ctrl_cmd_node")

        # Save Start time
        self.first_time = rospy.get_time()

        # Create a publisher that publishes CtrlCmd type messages to "/ctrl_cmd" topic
        self.erp42_ctrl_pub = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=3)

        # Steering limit value
        self.steering_bound = 28.2

    # Define the main function of the class
    def main(self):
        # Create a CtrlCmd message object
        self.ctrl_cmd_msg = CtrlCmd()
        # Save the current time
        self.second_time = rospy.get_time()

        # Set speed  0 ~ 23.8 -> 0 ~ 1
        speed = 15
        speed = max(0, min(speed, 23.8))

        # Steering angle (left: positive, right: negative)
        steering_deg = 10
        steering_deg = max(-self.steering_bound, min(steering_deg, self.steering_bound))

        # Change angle to radians
        steering_rad = steering_deg * pi / 180
        steering_deg = -steering_deg

        # Save the speed value in the message object
        self.ctrl_cmd_msg.accel = speed

        # Save the steering value in the message object
        self.ctrl_cmd_msg.steering = steering_rad

        # Update speed and steering values every 3 seconds and publish them
        if self.second_time - self.first >= 3:
            # Update the first time
            self.first_time = self.second_time
            self.erp42_ctrl_pub.publish(self.ctrl_cmd_msg)


# When the main script is executed, run the following code.
if __name__ == "__main__":
    try:
        # Create an object of the ERP42_ctrl_cmd class
        erp42_ctrl_cmd = ERP42_ctrl_cmd()

        while not rospy.is_shutdown():
            erp42_ctrl_cmd.main()

    except rospy.ROSInterruptException:
        pass
