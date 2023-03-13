#!/usr/bin/env python3

import rospy
from morai_msgs.msg import EgoVehicleStatus
import os

# Define Control class
class Control:
    # Control class constructor
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("ERP42_status_node")
        # Create a subscriber that listens to messages on "/Ego_topic" topic and sets the callback function to status_CB.
        self.obj_sub = rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_CB)
        
    # Callback function called when receiving messages from subscriber
    def status_CB(self, msg):
        # Initialize EgoVehicleStatus() message type for simulator
        self.status_msg = EgoVehicleStatus()
        # Save the received message in the status_msg variable
        self.status_msg = msg
        # Clear the terminal
        os.system("clear")
        print(f"------------------------------")
        # Print acceleration
        print(f"acceleration :")
        print(f"    x : {self.status_msg.acceleration.x}")
        print(f"    y : {self.status_msg.acceleration.y}")
        print(f"    z : {self.status_msg.acceleration.z}")
        print(f"position :")
        # Print position
        print(f"    x : {self.status_msg.position.x}")
        print(f"    y : {self.status_msg.position.y}")
        print(f"    z : {self.status_msg.position.z}")
        # Print ERP-42 velocity
        print(f"velocity :")
        print(f"    x : {self.status_msg.velocity.x}")
        print(f"    y : {self.status_msg.velocity.y}")
        print(f"    z : {self.status_msg.velocity.z}")
        # Print direction
        print(f"heading : {self.status_msg.heading}")
        # Print accel
        print(f"accel : {self.status_msg.accel}")
        # Print brake
        print(f"brake : {self.status_msg.brake}")
        # Print wheel angle
        print(f"wheel_angle : {self.status_msg.wheel_angle}")
        print(f"------------------------------")


if __name__ == "__main__":
    try:
        # Create Control class object
        control = Control()
        # Continuously call callback function in an infinite loop
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
