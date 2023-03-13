#!/usr/bin/env python3

import rospy
from morai_msgs.msg import GPSMessage
import os


# Define GPS class
class GPS:
    # GPS class constructor
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("gps_node")
        # Create a subscriber that listens to messages on "/gps" topic and sets the callback function to gps_CB.
        self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.gps_CB)

    # Callback function called when receiving messages from subscriber
    def gps_CB(self, msg):
        # Create a message object
        self.gps_msg = GPSMessage()
        # Store the received msg in traffic_msg
        self.gps_msg = msg

        # Clear the terminal
        os.system("clear")
        print(f"------------------------------")
        # Print latitude
        print(f"latitude : {self.gps_msg.latitude}")
        # Print longitude
        print(f"longitude : {self.gps_msg.longitude}")
        # Print altitude
        print(f"altitude : {self.gps_msg.altitude}")
        print(f"------------------------------")


# When the main script is executed, run the following code.
if __name__ == "__main__":
    # Create GPS class object
    gps = GPS()
    # Continuously call callback function in an infinite loop
    rospy.spin()
