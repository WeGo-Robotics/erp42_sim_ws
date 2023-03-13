#!/usr/bin/env python3
# python3

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import numpy as np
import cv2


# Define Camera class
class Camera:
    # Camera class constructor
    def __init__(self):
        # Connect OpenCV and ROS
        self.bridge = CvBridge()
        # Initialize ROS node
        rospy.init_node("camera_node")

        # Create a subscriber that listens to messages on "/image_jpeg/compressed" topic and sets the callback function to camera_CB.
        self.image_sub = rospy.Subscriber(
            "/image_jpeg/compressed", CompressedImage, self.camera_CB
        )

        # Create a publisher that publishes CompressedImage messages to "/camera/compressed" topic.
        self.image_pub = rospy.Publisher(
            "/camera/compressed", CompressedImage, queue_size=10
        )

    def camera_CB(self, msg):
        # Convert CompressedImage to numpy array
        self.sub_img_msg = CompressedImage()
        # Save the arrived message to sub_img_msg variable.
        self.sub_img_msg = msg
        # Convert the CompressedImage message to a numpy array.
        self.img = self.bridge.compressed_imgmsg_to_cv2(self.sub_img_msg)

        # Publish cv2 image as ROS Image message
        self.pub_img_msg = self.bridge.cv2_to_compressed_imgmsg(self.img)
        # Publish the converted message to "/camera/compressed" topic.
        self.image_pub.publish(self.pub_img_msg)


# When the main script is executed, run the following code.
if __name__ == "__main__":
    try:
        # Create Camera class object
        camera = Camera()
        # Continuously call callback function in an infinite loop
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
