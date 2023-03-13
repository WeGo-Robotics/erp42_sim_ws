#!/usr/bin/env python3

import rospy
import rospkg
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np


class CvtColor:
    def __init__(self):
        # Initialize the ROS node with name "image_node"
        rospy.init_node("image_node", anonymous=False)
        # Initialize a publisher for the filtered image with topic name "filter_img" and message type "Image"
        self.image_cvt_pub = rospy.Publisher("filter_img", Image, queue_size=10)
        self.rate = rospy.Rate(10)  # 10Hz

        # Initialize a ROS package object
        rospack = rospkg.RosPack()
        # Get the file path of the current ROS package
        self.file_path = rospack.get_path("opencv_edu")
        # Append "/scripts/" to the file path
        self.file_path += "/scripts/"

        # Set the file path of the input images
        self.lenna_path = self.file_path + "Lenna.png"
        self.brain_path = self.file_path + "brain.png"

    def main(self):
        # Read the input images using OpenCV
        self.lenna = cv2.imread(self.lenna_path, cv2.IMREAD_COLOR)
        self.brain = cv2.imread(self.brain_path, cv2.IMREAD_COLOR)
        # Define a 5x5 kernel with all values set to 1/25
        self.kernel = np.ones((5, 5), np.float32) / 25

        # -1 indicates that the output image should have the same depth as the input image
        self.f2d_lenna = cv2.filter2D(self.lenna, -1, self.kernel)
        self.f2d_brain = cv2.filter2D(self.brain, -1, self.kernel)

        # gaussian filtering
        self.gaussian_lenna = cv2.GaussianBlur(self.lenna, (5, 5), 0)
        self.gaussian_brain = cv2.GaussianBlur(self.brain, (5, 5), 0)

        # medain filtering
        self.median_lenna = cv2.medianBlur(self.lenna, 5)
        self.median_brain = cv2.medianBlur(self.brain, 5)

        # bilateral filtering
        self.bilateral_lenna = cv2.bilateralFilter(self.lenna, 15, 40, 100)
        self.bilateral_brain = cv2.bilateralFilter(self.brain, 15, 40, 100)

        # compare filtering
        self.compare_lenna = cv2.hconcat(
            [
                self.lenna,
                self.f2d_lenna,
                self.gaussian_lenna,
                self.median_lenna,
                self.bilateral_lenna,
            ]
        )
        # Resize the concatenated images
        self.resize_lenna = cv2.resize(self.compare_lenna, [1000, 200])
        # Concatenate the brain images horizontally for comparison
        self.compare_brain = cv2.hconcat(
            [
                self.brain,
                self.f2d_brain,
                self.gaussian_brain,
                self.median_brain,
                self.bilateral_brain,
            ]
        )
        # Resize the concatenated images
        self.resize_brain = cv2.resize(self.compare_brain, [1000, 200])
        # Concatenate the Lenna and brain images vertically
        self.rst = cv2.vconcat([self.resize_lenna, self.resize_brain])

        # Initialize bridge to convert between ROS and OpenCV images
        self.bridge = CvBridge()
        # Convert the processed image to ROS format
        image_msg = self.bridge.cv2_to_imgmsg(self.rst, "bgr8")
        self.image_cvt_pub.publish(image_msg)
        # Sleep to maintain the rate of the node
        self.rate.sleep()


if __name__ == "__main__":
    cvtColor = CvtColor()
    try:
        while not rospy.is_shutdown():
            cvtColor.main()
    except rospy.ROSInterruptException:
        pass
