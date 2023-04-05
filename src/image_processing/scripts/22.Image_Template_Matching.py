#!/usr/bin/env python3

# import libraries
import cv2  # OpenCV library for image processing
import rospy  # ROS Python library for working with ROS
from sensor_msgs.msg import Image  # ROS Image message type
from cv_bridge import CvBridge  # Library to convert between ROS and OpenCV images
import rospkg  # ROS package library
import numpy as np
from matplotlib import pyplot as plt


class Image_Template_Matching:
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
        self.face_image_path = self.file_path + "messi_Face.png"
        self.body_image_path = self.file_path + "messi.jpeg"
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
        img = cv2.imread(self.body_image_path)
        img2 = img.copy()
        template = cv2.imread(self.face_image_path)
        w, h = template.shape[:2][::-1]
        # Template Match Method
        methods = [
            "cv2.TM_CCOEFF",
            "cv2.TM_CCOEFF_NORMED",
            "cv2.TM_CCORR",
            "cv2.TM_CCORR_NORMED",
            "cv2.TM_SQDIFF",
            "cv2.TM_SQDIFF_NORMED",
        ]
        for method in methods:
            img = img2.copy()
            method = eval(method)
            res = cv2.matchTemplate(img, template, method)
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
            if method in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]:
                top_left = min_loc
            else:
                top_left = max_loc
            bottom_right = (top_left[0] + w, top_left[1] + h)
            cv2.rectangle(img, top_left, bottom_right, 255, 5)
            # plt.subplot(121), plt.title(method), plt.imshow(res, cmap="gray"), plt.yticks(
            # []
            # ), plt.xticks([])
            # plt.subplot(122), plt.imshow(img, cmap="gray")
            # plt.show()


if __name__ == "__main__":
    # Create an instance of the ImagePublisher class and run it
    image_template_matching = Image_Template_Matching()
    try:
        image_template_matching.main()

        while not rospy.is_shutdown():
            image_template_matching.main()

    except rospy.ROSInterruptException:
        pass
