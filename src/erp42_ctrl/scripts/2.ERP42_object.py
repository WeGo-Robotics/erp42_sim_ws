#!/usr/bin/env python3

import rospy
from morai_msgs.msg import ObjectStatusList
import os


# Define Object class
class Object:
    # Object class constructor
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("object_node")
        self.obj_sub = rospy.Subscriber(
            # Create a subscriber that listens to messages on "/Object_topic" topic and sets the callback function to object_CB.
            "/Object_topic",
            ObjectStatusList,
            self.object_CB,
        )

    # Callback function called when receiving messages from subscriber
    def object_CB(self, msg):
        # Create a message object
        self.obj_msg = ObjectStatusList()

        # Save the received message in the obj_msg variable
        self.obj_msg = msg

        # Clear the terminal
        os.system("clear")

        # Loop through each obstacle in the received message's obstacle_list
        for obj in self.obj_msg.obstacle_list:
            # Print the type of the current obstacle
            print(f"Object ID: {obj.unique_id}")

            # Print the position of the current obstacle
            print(f"Object Type: {obj.type}")

            # Indent the next three lines to make it clear that they are part of the "Object Pos" section
            print(f"Object Pos: ")
            print(f"    x : {obj.position.x}")
            print(f"    y : {obj.position.y}")
            print(f"    z : {obj.position.z}")


# When the main script is executed, run the following code.
if __name__ == "__main__":
    try:
        object = Object()

        # Continuously call callback function in an infinite loop
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
