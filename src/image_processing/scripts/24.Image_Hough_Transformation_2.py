#!/usr/bin/env python3

# import libraries
import cv2  # OpenCV library for image processing
import rospy  # ROS Python library for working with ROS
from sensor_msgs.msg import Image  # ROS Image message type
from cv_bridge import CvBridge  # Library to convert between ROS and OpenCV images
import rospkg  # ROS package library
import numpy as np


class Image_Hough_Transformation2:
    def __init__(self):
        # initialize a ROS node
        rospy.init_node("image_node")

        # define a list of titles for the images to be displayed
        self.titles = ["image", "hough_transform2"]

        # set the publishing rate to 10 Hz
        self.rate = rospy.Rate(10)

        # Get the file path of the "image_processing" package
        rospack = rospkg.RosPack()
        self.file_path = rospack.get_path("image_processing")
        self.file_path += "/scripts/"

        # Set the file path for the image file to be used
        self.image_path = self.file_path + "hough_trans_test.jpeg"
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
        # Load the input image
        image = cv2.imread(self.image_path)
        image_ori = image.copy()
        # Detect edges in the input image using the Canny edge detection algorithm
        edges = cv2.Canny(image, 240, 255, apertureSize=3)
        # Convert the grayscale image to a color image
        gray = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        # Set the minimum length of a line segment to be detected
        minLineLength = 100
        # Set the maximum allowed gap between line segments to be considered as a single line
        maxLineGap = 3
        # Apply Hough transform to detect lines in the edge image
        lines = cv2.HoughLinesP(edges, 1, np.pi / 360, 30, minLineLength, maxLineGap)
        # Iterate through the detected lines
        for i in range(len(lines)):
            # Get the endpoints of the line segment
            for x1, y1, x2, y2 in lines[i]:
                # Draw the line segment on the original image
                cv2.line(image, (x1, y1), (x2, y2), (0, 0, 255), 3)

        images = [image_ori, image]

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
    image_hough_transformation2 = Image_Hough_Transformation2()

    try:
        # run the main function of the Image_Histogram_CLAHE class until the ROS node is shut down
        while not rospy.is_shutdown():
            image_hough_transformation2.main()

    except rospy.ROSInterruptException:
        pass
