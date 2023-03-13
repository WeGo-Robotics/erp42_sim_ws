#!/usr/bin/env python3

import rospy
import rospkg
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np


# Define the Binary class
class Binary:
    # Constructor method, called when an instance of the class is created
    def __init__(self):
        # Initialize a ROS node called "image_node"
        rospy.init_node("image_node", anonymous=False)
        # Create a ROS publisher that publishes binary images to the "binary_img" topic
        self.image_pub = rospy.Publisher("binary_img", Image, queue_size=10)
        self.rate = rospy.Rate(10)  # 10Hz

        # Get the path to the opencv_edu package
        rospack = rospkg.RosPack()
        self.file_path = rospack.get_path("opencv_edu")
        self.file_path += "/scripts/"
        # Set the path to the input image
        self.wego_path = self.file_path + "wego.png"

    # Main method that performs the image processing and publishing
    def main(self):
        # Initialize bridge to convert between ROS and OpenCV images
        bridge = CvBridge()
        # Load the input image in color
        self.wego = cv2.imread(self.wego_path, cv2.IMREAD_COLOR)
        # Convert the color image to grayscale
        wego_gray = cv2.cvtColor(self.wego, cv2.COLOR_BGR2GRAY)
        # Apply a binary threshold to the grayscale image to get a binary image
        _, wego_binary = cv2.threshold(wego_gray, 127, 255, cv2.THRESH_BINARY)
        # Apply a binary inverse threshold to the grayscale image to get an inverted binary image
        _, wego_binary_INV = cv2.threshold(wego_gray, 127, 255, cv2.THRESH_BINARY_INV)
        # Concatenate the binary and inverted binary images horizontally to create a comparison image
        con_binary = cv2.hconcat((wego_binary, wego_binary_INV))
        # Resize the comparison image to match the size of the grayscale image
        resize_con_binary = cv2.resize(
            con_binary, (wego_gray.shape[1], wego_gray.shape[0])
        )
        # Concatenate the grayscale image and the resized comparison image vertically
        thresh_rst = cv2.vconcat((wego_gray, resize_con_binary))
        # Convert the concatenated image to a ROS message
        image_msg = bridge.cv2_to_imgmsg(thresh_rst)
        # Publish the image message to the "binary_img" topic
        self.image_pub.publish(image_msg)
        # Wait for the next publishing cycle
        self.rate.sleep()


if __name__ == "__main__":
    binary = Binary()
    try:
        while not rospy.is_shutdown():
            binary.main()
    except rospy.ROSInterruptException:
        pass
