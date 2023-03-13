#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


# Define a class called Approach_Index
class Approach_Index:
    # Define the initialization function of the class
    def __init__(self):
        # Initialize a ROS node called "image_node"
        rospy.init_node("image_node", anonymous=False)
        # Create a publisher to publish the image
        self.color_img_pub = rospy.Publisher("color_img", Image, queue_size=10)
        self.grayscale_img_pub = rospy.Publisher("gray_img", Image, queue_size=10)
        # Set the height and width of the images
        self.height = 150
        self.width = 150
        # Create a 3-channel black image with the specified height and width
        self.color_space = np.zeros((self.height, self.width, 3), np.uint8)
        # Create a single-channel black image with the specified height and width
        self.grayscale_space = np.zeros((self.height, self.width), np.uint8)
        
    # Define the main function of the class
    def main(self):
        # Initialize bridge to convert between ROS and OpenCV images
        bridge = CvBridge()

        # Define the colors blue, green and red as lists of RGB values
        blue = [255, 0, 0]
        green = [0, 255, 0]
        red = [0, 0, 255]

        # Set a pixel in the center of the color image to blue
        self.color_space[75, 75] = blue
        # Set a rectangular region in the color image to green
        self.color_space[0:50, 50:100] = green
        # Set a rectangular region at the bottom of the color image to red
        self.color_space[-10:-1, :] = red

        # Set a pixel in the center of the grayscale image to white
        self.grayscale_space[75, 75] = 255
        # Set a rectangular region in the grayscale image to a medium gray color
        self.grayscale_space[0:50, 50:100] = 150

        # Convert the color image from OpenCV format to ROS format
        image_color = bridge.cv2_to_imgmsg(self.color_space, "bgr8")
        # Convert the grayscale image from OpenCV format to ROS format
        image_grayscale = bridge.cv2_to_imgmsg(self.grayscale_space)
        # Publish the color image on the "color_img" topic
        self.color_img_pub.publish(image_color)
        # Publish the grayscale image on the "gray_img" topic
        self.grayscale_img_pub.publish(image_grayscale)


if __name__ == "__main__":
    approach_index = Approach_Index()
    try:
        while not rospy.is_shutdown():
            approach_index.main()
    except rospy.ROSInterruptException:
        pass
