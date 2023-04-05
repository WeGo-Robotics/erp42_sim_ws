#!/usr/bin/env python3

# import libraries
import cv2  # OpenCV library for image processing
import rospy  # ROS Python library for working with ROS
from sensor_msgs.msg import Image  # ROS Image message type
from cv_bridge import CvBridge  # Library to convert between ROS and OpenCV images
import rospkg  # ROS package library
import numpy as np


class Image_Histogram_2d:
    def __init__(self):
        # initialize a ROS node
        rospy.init_node("image_node")

        # define a list of titles for the images to be displayed
        self.titles = ["image", "2d_histogram"]

        # set the publishing rate to 10 Hz
        self.rate = rospy.Rate(10)

        # Get the file path of the "image_processing" package
        rospack = rospkg.RosPack()
        self.file_path = rospack.get_path("image_processing")
        self.file_path += "/scripts/"

        # Set the file path for the image file to be used
        self.image_path = self.file_path + "home.jpg"
        # Initialize bridge to convert between ROS and OpenCV images
        self.bridge = CvBridge()
        # create a list of publishers for each of the four transformed images
        self.pubs = []

        for title in self.titles:
            # create a publisher for each title in the titles list
            pub = rospy.Publisher("/image_topic/" + title, Image, queue_size=1)
            # append the publisher to the pubs list
            self.pubs.append(pub)

    def main(self):
        # Load an image from the file path using OpenCV
        image = cv2.imread(self.image_path)
        # Convert the color space from BGR to HSV using OpenCV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # Compute the 2D histogram of the Hue-Saturation plane using OpenCV's calcHist function
        # The histogram will have 180 bins for the Hue channel and 256 bins for the Saturation channel
        # The range for the Hue channel is [0,180], and the range for the Saturation channel is [0,256]
        hist = cv2.calcHist([hsv], [0, 1], None, [180, 256], [0, 180, 0, 256])

        images = [image, hist]

        # publish each image to its corresponding topic
        for i, image in enumerate(images):
            # Convert the OpenCV image to a ROS message
            if len(image.shape) == 2:
                # Grayscale image
                ros_image = self.bridge.cv2_to_imgmsg(image)
            else:
                # Color image
                ros_image = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")

            # publish the ROS image message to thae corresponding topic
            self.pubs[i].publish(ros_image)

            # wait for the next publishing cycle
            self.rate.sleep()


# create an instance of the Image_Transformation_Traslation class
if __name__ == "__main__":
    image_histogram_2d = Image_Histogram_2d()
    try:
        # run the main function of the Image_Transformation_Traslation class until the ROS node is shut down
        while not rospy.is_shutdown():
            image_histogram_2d.main()

    except rospy.ROSInterruptException:
        pass
