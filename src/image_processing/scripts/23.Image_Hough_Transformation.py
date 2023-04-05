#!/usr/bin/env python3

# import libraries
import cv2  # OpenCV library for image processing
import rospy  # ROS Python library for working with ROS
from sensor_msgs.msg import Image  # ROS Image message type
from cv_bridge import CvBridge  # Library to convert between ROS and OpenCV images
import rospkg  # ROS package library
import numpy as np


class Image_Hough_Transformation:
    def __init__(self):
        # initialize a ROS node
        rospy.init_node("image_node")

        # define a list of titles for the images to be displayed
        self.titles = ["image", "hough_transform"]

        # set the publishing rate to 10 Hz
        self.rate = rospy.Rate(10)

        # Get the file path of the "image_processing" package
        rospack = rospkg.RosPack()
        self.file_path = rospack.get_path("image_processing")
        self.file_path += "/scripts/"

        # Set the file path for the image file to be used
        self.image_path = self.file_path + "Hough_transform.png"

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
        # Load the image from file
        image = cv2.imread(self.image_path)

        # Create a copy of the original image
        image_original = image.copy()

        # Convert the image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Detect edges in the image using the Canny edge detection algorithm
        edges = cv2.Canny(gray, 50, 150, apertureSize=3)

        # Detect lines in the image using the Hough transform algorithm
        lines = cv2.HoughLines(edges, 1, np.pi / 180, 90)

        # Draw the detected lines on the original image
        for i in range(len(lines)):
            for rho, theta in lines[i]:
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * (a))
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * (a))
                cv2.line(image, (x1, y1), (x2, y2), (0, 0, 255), 2)

        images = [image_original, image]
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


# create an instance of the Image_Histogram_CLAHE class
if __name__ == "__main__":
    image_hough_transformation = Image_Hough_Transformation()

    try:
        # run the main function of the Image_Histogram_CLAHE class until the ROS node is shut down
        while not rospy.is_shutdown():
            image_hough_transformation.main()

    except rospy.ROSInterruptException:
        pass
