#!/usr/bin/env python3

# import libraries
import cv2  # OpenCV library for image processing
import numpy as np  # NumPy library for numerical operations
import rospy  # ROS Python library for working with ROS
from sensor_msgs.msg import Image  # ROS Image message type
from cv_bridge import CvBridge  # Library to convert between ROS and OpenCV images
import rospkg  # ROS package library


# create a class for image transformation and zoom
class Image_Transformation_Traslation:
    def __init__(self):
        # initialize a ROS node
        rospy.init_node("image_node")

        # define a list of titles for the images to be displayed
        self.titles = ["original", "translation"]

        # set the publishing rate to 10 Hz
        self.rate = rospy.Rate(10)

        # Get the file path of the "image_processing" package
        rospack = rospkg.RosPack()
        self.file_path = rospack.get_path("image_processing")
        self.file_path += "/scripts/"

        # Set the file path for the image file to be used
        self.image_path = self.file_path + "opencv-logo.png"

        # Initialize bridge to convert between ROS and OpenCV images
        self.bridge = CvBridge()

        # create a list of publishers for each of the four transformed images
        self.pubs = []
        for title in self.titles:
            pub = rospy.Publisher("/image_topic/" + title, Image, queue_size=1)
            self.pubs.append(pub)

    def main(self):
        # read image
        image = cv2.imread(self.image_path)

        # Transform image
        rows, cols = image.shape[:2]
        M = np.float32([[1, 0, 30], [0, 1, 60]])  # Translation matrix
        transformed = cv2.warpAffine(image, M, (cols, rows))  # translate image

        # create a list of the original image and the transformed images
        images = [image, transformed]

        # publish each image to its corresponding topic
        for i, image in enumerate(images):
            # convert the OpenCV image to a ROS image message
            ros_img = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")

            # publish the ROS image message to the corresponding topic
            self.pubs[i].publish(ros_img)

            # wait for the next publishing cycle
            self.rate.sleep()


# create an instance of the Image_Transformation_Traslation class
if __name__ == "__main__":
    image_transformation_traslation = Image_Transformation_Traslation()
    try:
        # run the main function of the Image_Transformation_Traslation class until the ROS node is shut down
        while not rospy.is_shutdown():
            image_transformation_traslation.main()

    except rospy.ROSInterruptException:
        pass
