#!/usr/bin/env python3

import rospy
from morai_msgs.msg import GetTrafficLightStatus
import os


# Define Get_traffic class
class Get_traffic:
    # Get_traffic class constructor
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("get_traffic_node")

        # Create a subscriber that listens to messages on "/GetTrafficLightStatus" topic and sets the callback function to traffic_CB.
        self.get_traffic_sub = rospy.Subscriber(
            "/GetTrafficLightStatus", GetTrafficLightStatus, self.traffic_CB
        )

    # Callback function called when receiving messages from subscriber
    def traffic_CB(self, msg):
        # Create a message object
        traffic_msg = GetTrafficLightStatus()
        # Save the received message in the traffic_msg
        traffic_msg = msg

        # Clear the terminal
        os.system("clear")
        print(f"------------------------------")
        # Print the traffic light index value
        print(f"trafficLightIndex : {traffic_msg.trafficLightIndex}")

        # Print the traffic light type
        print(f"trafficLightType : {traffic_msg.trafficLightType}")

        # Print the traffic light status
        print(f"trafficLightStatus : {traffic_msg.trafficLightStatus}")
        print(f"------------------------------")


# When the main script is executed, run the following code.
if __name__ == "__main__":
    try:
        # Create Get_traffic class object
        get_traffic = Get_traffic()
        # Continuously call callback function in an infinite loop
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
