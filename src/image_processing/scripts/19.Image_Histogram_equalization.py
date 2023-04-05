#!/usr/bin/env python3

# import libraries
import cv2  # OpenCV library for image processing
import rospy  # ROS Python library for working with ROS
from sensor_msgs.msg import Image  # ROS Image message type
from cv_bridge import CvBridge  # Library to convert between ROS and OpenCV images
import rospkg  # ROS package library
import numpy as np


class Image_Histogram_Equalization:
    def __init__(self):
        # initialize a ROS node
        rospy.init_node("image_node")

        # define a list of titles for the images to be displayed
        self.titles = ["image", "equalization_image"]

        # set the publishing rate to 10 Hz
        self.rate = rospy.Rate(10)

        # Get the file path of the "image_processing" package
        rospack = rospkg.RosPack()
        self.file_path = rospack.get_path("image_processing")
        self.file_path += "/scripts/"

        # Set the file path for the image file to be used
        self.image_path = self.file_path + "equalization.jpg"
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
        # Load the image
        image = cv2.imread(self.image_path, 0)

        # Apply CLAHE (Contrast Limited Adaptive Histogram Equalization)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        image2 = clahe.apply(image)

        # Resize the images for display
        image = cv2.resize(image, (400, 400))
        image2 = cv2.resize(image2, (400, 400))

        # # Combine the original and enhanced images horizontally for display
        # dst = np.hstack((image, image2))

        images = [image, image2]

        # publish each image to its corresponding topic
        for i, image in enumerate(images):
            # convert the OpenCV image to a ROS image message
            ros_img = self.bridge.cv2_to_imgmsg(image)

            # publish the ROS image message to thae corresponding topic
            self.pubs[i].publish(ros_img)

            # wait for the next publishing cycle
            self.rate.sleep()


# create an instance of the Image_Transformation_Traslation class
if __name__ == "__main__":
    image_histogram_equalization = Image_Histogram_Equalization()
    try:
        # run the main function of the Image_Transformation_Traslation class until the ROS node is shut down
        while not rospy.is_shutdown():
            image_histogram_equalization.main()

    except rospy.ROSInterruptException:
        pass
