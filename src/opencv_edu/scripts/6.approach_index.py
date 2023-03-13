#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class Approach_Index:
    def __init__(self):
        rospy.init_node("image_node", anonymous=False)
        # Create a publisher to publish the image
        self.color_img_pub = rospy.Publisher("color_img", Image, queue_size=10)
        self.grayscale_img_pub = rospy.Publisher("gray_img", Image, queue_size=10)
        self.height = 150
        self.width = 150
        self.color_space = np.zeros((self.height, self.width, 3), np.uint8)
        self.grayscale_space = np.zeros((self.height, self.width), np.uint8)

    def main(self):
        # Initialize bridge to convert between ROS and OpenCV images
        bridge = CvBridge()

        blue = [255, 0, 0]
        green = [0, 255, 0]
        red = [0, 0, 255]

        self.color_space[75, 75] = blue
        self.color_space[0:50, 50:100] = green
        self.color_space[-10:-1, :] = red

        self.grayscale_space[75, 75] = 255
        self.grayscale_space[0:50, 50:100] = 150

        image_color = bridge.cv2_to_imgmsg(self.color_space, "bgr8")
        image_grayscale = bridge.cv2_to_imgmsg(self.grayscale_space)
        self.color_img_pub.publish(image_color)
        self.grayscale_img_pub.publish(image_grayscale)


if __name__ == "__main__":
    approach_index = Approach_Index()
    try:
        while not rospy.is_shutdown():
            approach_index.main()
    except rospy.ROSInterruptException:
        pass
