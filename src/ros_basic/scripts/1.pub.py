#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
import os


# Define the initialization function of the class
class Pubish:
    # Defines the constructor for the Pubish class
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("publish_node")

        # Creates a publisher object that publishes to the "count" topic using the Int32 message type, with a queue size of 1.
        self.pub = rospy.Publisher("count", Int32, queue_size=1)

        # Initialize the value to 0.
        self.count = 0

        # Set the publishing rate to 10
        self.rate = rospy.Rate(10)

    def main(self):
        # Create a Int32 message object
        self.Int32_msg = Int32()

        # Publish the ROS Int32 message
        self.pub.publish(self.count)

        # Prints the current count value.
        os.system("clear")
        print(f"------------------------------")
        print(f"count : ", self.count)
        print(f"------------------------------")
        # Sleep for a certain duration to maintain the publishing rate
        self.rate.sleep()

        # Increments the count value by 1.
        self.count += 1


# Executes the following code only if the script is being run as the main program.
if __name__ == "__main__":
    #  Ignores the exception if a ROSInterruptException is raised.
    try:
        # Create Pubish class object
        publish = Pubish()

        # Runs a loop as long as ROS is not shutdown.
        while not rospy.is_shutdown():
            # Executes the main function of the Pubish class.
            publish.main()

    except rospy.ROSInterruptException:
        pass
