#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import numpy as np
import cv2


# Define Camera class
class Camera:
    # Camera class constructor
    def __init__(self):
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
        # Connect OpenCV and ROS
        self.bridge = CvBridge()

    # Callback function called when receiving messages from subscriber
    def camera_CB(self, msg):
        #  Initialize CompressedImage() message type for simulator
        sub_img_msg = CompressedImage()

        # Save the received message in the sub_img_msg
        sub_img_msg = msg

        # Convert ROS image message to a OpenCV image
        self.img_msg = self.bridge.compressed_imgmsg_to_cv2(sub_img_msg)

        # Convert the OpenCV image to a ROS image message
        self.pub_img_msg = self.bridge.cv2_to_compressed_imgmsg(self.img_msg)

        # Publish cv2 image as ROS Image message
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
