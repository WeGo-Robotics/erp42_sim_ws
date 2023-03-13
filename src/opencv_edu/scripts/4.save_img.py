#!/usr/bin/env python3

import rospy
import rospkg
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np


# Define a class called Color_And_Gray
class Color_And_Gray:
    def __init__(self):
        # Initialize a ROS node with the name "image_node"
        rospy.init_node("image_node", anonymous=False)
        # Initialize publishers to publish images in color and grayscale
        self.image_pub_color = rospy.Publisher("bgr_img", Image, queue_size=10)
        self.image_pub_gray = rospy.Publisher("gray_img", Image, queue_size=10)
        self.rate = rospy.Rate(10)  # 10Hz

        # Get the path of the current ROS package
        rospack = rospkg.RosPack()
        self.file_path = rospack.get_path("opencv_edu")
        self.file_path += "/scripts/"

        # Set the file path for the Wego image
        self.wego_path = self.file_path + "wego.png"

    def main(self):
        # Initialize bridge to convert between ROS and OpenCV images
        bridge = CvBridge()
        # Read the Wego image in BGR format using OpenCV
        self.wego_bgr = cv2.imread(self.wego_path, cv2.IMREAD_COLOR)

        # Convert from color images to gray images
        self.wego_gray = cv2.cvtColor(self.wego_bgr, cv2.COLOR_BGR2GRAY)

        # Write the color and grayscale images to files
        cv2.imwrite(self.file_path + "wego_color.png", self.wego_bgr)
        cv2.imwrite(self.file_path + "wego_color.png", self.wego_gray)
        # Convert the OpenCV images to ROS image messages
        ros_image_color = bridge.cv2_to_imgmsg(self.wego_bgr, "bgr8")
        ros_image_gray = bridge.cv2_to_imgmsg(self.wego_gray)

        # Publish the ROS image messages
        self.image_pub_color.publish(ros_image_color)
        self.image_pub_gray.publish(ros_image_gray)

        # Wait for the next loop iteration
        self.rate.sleep()


if __name__ == "__main__":
    color_and_gray = Color_And_Gray()
    try:
        while not rospy.is_shutdown():
            color_and_gray.main()
    except rospy.ROSInterruptException:
        pass
