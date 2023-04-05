#!/usr/bin/env python3

# import libraries
import cv2  # OpenCV library for image processing
import numpy as np  # NumPy library for numerical operations
import rospy  # ROS Python library for working with ROS
from sensor_msgs.msg import Image  # ROS Image message type
from cv_bridge import CvBridge  # Library to convert between ROS and OpenCV images
import rospkg  # ROS package library


class Canny_Edge_Detection:
    def __init__(self):
        # initialize a ROS node
        rospy.init_node("image_node")

        # Define titles for each edge detection method
        self.titles = ["canny", "laplacian", "sobelx", "sobely"]

        # set the publishing rate to 10 Hz
        self.rate = rospy.Rate(10)

        # Get the file path of the "image_processing" package
        rospack = rospkg.RosPack()
        self.file_path = rospack.get_path("image_processing")
        self.file_path += "/scripts/"

        # Set the file path for the image file to be used
        self.image_path = self.file_path + "sudoku.png"

        # Initialize bridge to convert between ROS and OpenCV images
        self.bridge = CvBridge()

        # create a list of publishers for each of the four transformed images
        self.pubs = []
        for title in self.titles:
            pub = rospy.Publisher("/image_topic/" + title, Image, queue_size=1)
            self.pubs.append(pub)

    def main(self):
        image = cv2.imread(self.image_path, 0)

        canny = cv2.Canny(image, 30, 70)
        laplacian = cv2.Laplacian(image, cv2.CV_8U)
        sobelx = cv2.Sobel(image, cv2.CV_8U, 1, 0, ksize=3)
        sobely = cv2.Sobel(image, cv2.CV_8U, 0, 1, ksize=3)
        # Store all the edge detection images in a list
        images = [canny, laplacian, sobelx, sobely]

        # publish each image to its corresponding topic
        for i, image in enumerate(images):
            # convert the OpenCV image to a ROS image message
            ros_img = self.bridge.cv2_to_imgmsg(image)

            # publish the ROS image message to the corresponding topic
            self.pubs[i].publish(ros_img)

            # wait for the next publishing cycle
            self.rate.sleep()


# create an instance of the Image_Transformation_Traslation class
if __name__ == "__main__":
    canny_edge_detection = Canny_Edge_Detection()
    try:
        # run the main function of the Image_Transformation_Traslation class until the ROS node is shut down
        while not rospy.is_shutdown():
            canny_edge_detection.main()

    except rospy.ROSInterruptException:
        pass
