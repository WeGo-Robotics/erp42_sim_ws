#!/usr/bin/env python3

# import libraries
import cv2  # OpenCV library for image processing
import rospy  # ROS Python library for working with ROS
from sensor_msgs.msg import Image  # ROS Image message type
from cv_bridge import CvBridge  # Library to convert between ROS and OpenCV images
import rospkg  # ROS package library
import numpy as np


class Image_Pyramid:
    def __init__(self):
        # initialize a ROS node
        rospy.init_node("image_node")

        # define a list of titles for the images to be displayed
        self.titles = ["real", "ls"]

        # set the publishing rate to 10 Hz
        self.rate = rospy.Rate(10)

        # Get the file path of the "image_processing" package
        rospack = rospkg.RosPack()
        self.file_path = rospack.get_path("image_processing")
        self.file_path += "/scripts/"

        # Set the file path for the image file to be used
        self.A_image_path = self.file_path + "apple.jpg"
        self.B_image_path = self.file_path + "orange.jpg"
        # Initialize bridge to convert between ROS and OpenCV images
        self.bridge = CvBridge()
        # create a list of publishers for each of the four transformed images
        self.pubs = []
        for title in self.titles:
            # create a publisher for each title in the titles list
            pub = rospy.Publisher("/image_topic/" + title, Image, queue_size=1)
            # append the publisher to the pubs list
            self.pubs.append(pub)

    def generate_gaussian_pyramid(self, img, levels):
        pyramid = [img.copy()]
        for i in range(levels):
            img = cv2.pyrDown(img)
            pyramid.append(img)
        return pyramid

    def generate_laplacian_pyramid(self, gaussian_pyramid):
        pyramid = [gaussian_pyramid[-1]]
        for i in range(len(gaussian_pyramid) - 1, 0, -1):
            GE = cv2.pyrUp(gaussian_pyramid[i])
            L = cv2.subtract(
                gaussian_pyramid[i - 1],
                cv2.resize(GE, gaussian_pyramid[i - 1].shape[:2][::-1]),
            )
            pyramid.append(L)
        return pyramid

    def main(self):
        A_image = cv2.imread(self.A_image_path)
        B_image = cv2.imread(self.B_image_path)
        # generate Gaussian pyramid for A and B
        gpA = self.generate_gaussian_pyramid(A_image, 6)
        gpB = self.generate_gaussian_pyramid(B_image, 6)
        # generate Laplacian Pyramid for A and B
        lpA = self.generate_laplacian_pyramid(gpA)
        lpB = self.generate_laplacian_pyramid(gpB)
        # add left and right halves of images in each level
        LS = []
        cols = A_image.shape[1]
        for la, lb in zip(lpA, lpB):
            rows, cols, dpt = la.shape
            ls = np.hstack((la[:, : int(cols / 2)], lb[:, int(cols / 2) :]))
            LS.append(ls)
        # reconstruct
        ls_ = LS[0]
        for i in range(1, 6):
            ls_ = cv2.pyrUp(ls_)
            ls_ = cv2.add(ls_[: LS[i].shape[0], : LS[i].shape[1], :], LS[i])
        # image with direct connecting each half
        real = np.hstack((A_image[:, : int(cols / 2)], B_image[:, int(cols / 2) :]))
        images = [real, ls_]
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
    image_pyramid = Image_Pyramid()
    try:
        # run the main function of the Image_Transformation_Traslation class until the ROS node is shut down
        while not rospy.is_shutdown():
            image_pyramid.main()

    except rospy.ROSInterruptException:
        pass
