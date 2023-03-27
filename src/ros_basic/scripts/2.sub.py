#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
import os


# Define the initialization function of the class
class Subscribe:
    # Defines the constructor for the Pubish class
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("subscribe_node")
        # Create a subscriber object that subscribes to the "count" topic using the Int32 message type.
        self.sub = rospy.Subscriber("count", Int32, callback=self.count_CB)

    # Define the count_CB method for the Subscribe class. This method is called whenever a message is received on the "count" topic.
    def count_CB(self, msg):
        os.system("clear")
        print(f"------------------------------")
        print(f"msg : ", msg)
        print(f"msg.data : ", msg.data)
        print(f"------------------------------")


if __name__ == "__main__":
    try:
        # Create an object of the Subscribe class.
        subscribe = Subscribe()

        # Continuously call callback function in an infinite loop
        rospy.spin()

    # Ignore the exception if a ROSInterruptException is raised.
    except rospy.ROSInterruptException:
        pass
