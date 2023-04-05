#!/usr/bin/env python3

# import libraries
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospkg


class Image_Morphological_Transformation:
    def __init__(self):
        # initialize a ROS node
        rospy.init_node("image_node")

        # define a list of titles for the images to be displayed
        self.titles = [
            "dotImage",
            "erosion",
            "opening",
            "holeImage",
            "dilation",
            "closing",
            "gradient",
            "tophat",
            "blackhat",
        ]

        # set the publishing rate to 10 Hz
        self.rate = rospy.Rate(10)

        # Get the file path of the "image_processing" package
        rospack = rospkg.RosPack()
        self.file_path = rospack.get_path("image_processing")
        self.file_path += "/scripts/"

        # Set the file path for the image file to be used
        self.image_path = self.file_path + "j.png"

        # Initialize bridge to convert between ROS and OpenCV images
        self.bridge = CvBridge()

        # create a list of publishers for each of the four transformed images
        self.pubs = []
        for title in self.titles:
            pub = rospy.Publisher("/image_topic/" + title, Image, queue_size=1)
            self.pubs.append(pub)

    def main(self):
        dotImage = cv2.imread(self.image_path)
        holeImage = cv2.imread(self.image_path)
        orig = cv2.imread(self.image_path)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

        erosion = cv2.erode(dotImage, kernel, iterations=1)
        dilation = cv2.dilate(holeImage, kernel, iterations=1)
        opening = cv2.morphologyEx(dotImage, cv2.MORPH_OPEN, kernel)
        closing = cv2.morphologyEx(holeImage, cv2.MORPH_CLOSE, kernel)
        gradient = cv2.morphologyEx(orig, cv2.MORPH_GRADIENT, kernel)
        tophat = cv2.morphologyEx(orig, cv2.MORPH_TOPHAT, kernel)
        blackhat = cv2.morphologyEx(orig, cv2.MORPH_BLACKHAT, kernel)

        images = [
            dotImage,
            erosion,
            opening,
            holeImage,
            dilation,
            closing,
            gradient,
            tophat,
            blackhat,
        ]

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
    image_morphological_transformation = Image_Morphological_Transformation()
    try:
        # run the main function of the Image_Transformation_Traslation class until the ROS node is shut down
        while not rospy.is_shutdown():
            image_morphological_transformation.main()

    except rospy.ROSInterruptException:
        pass
