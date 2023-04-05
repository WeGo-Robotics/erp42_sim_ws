#!/usr/bin/env python3

# import libraries
import cv2  # OpenCV library for image processing
import rospy  # ROS Python library for working with ROS
from sensor_msgs.msg import Image  # ROS Image message type
from cv_bridge import CvBridge  # Library to convert between ROS and OpenCV images
import rospkg  # ROS package library
import numpy as np

from matplotlib import pyplot as plt


class Image_Histogram:
    def __init__(self):
        # initialize a ROS node
        rospy.init_node("image_node")

        # define a list of titles for the images to be displayed
        self.titles = ["Red_flower", "Red_histogram", "white_flower", "white_histogram"]

        # set the publishing rate to 10 Hz
        self.rate = rospy.Rate(10)

        # Get the file path of the "image_processing" package
        rospack = rospkg.RosPack()
        self.file_path = rospack.get_path("image_processing")
        self.file_path += "/scripts/"

        # Set the file path for the image file to be used
        self.Red_image_path = self.file_path + "red_rose.jpg"
        self.Green_image_path = self.file_path + "white_lotus.jpg"
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
        # Load the two images to be processed
        image1 = cv2.imread(self.Red_image_path)
        image2 = cv2.imread(self.Green_image_path)

        # Compute histograms of the two images
        hist1 = cv2.calcHist([image1], [0], None, [256], [0, 256])
        hist2 = cv2.calcHist([image2], [0], None, [256], [0, 256])

        # Create an image from the histogram data for the first image
        hist_img1 = np.zeros((256, 256, 3), dtype=np.uint8)
        hist_normalized = cv2.normalize(
            hist1, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX
        )
        for i in range(256):
            cv2.line(
                hist_img1,
                (i, 255),
                (i, 255 - int(hist_normalized[i])),
                color=(255, 255, 255),
                thickness=1,
            )

        # Create an image from the histogram data for the second image
        hist_img2 = np.zeros((256, 256, 3), dtype=np.uint8)
        hist_normalized = cv2.normalize(
            hist2, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX
        )
        for i in range(256):
            cv2.line(
                hist_img2,
                (i, 255),
                (i, 255 - int(hist_normalized[i])),
                color=(255, 255, 255),
                thickness=1,
            )

        # Create a list of the four images to be published
        images = [image1, hist_img1, image2, hist_img2]

        # plt.subplot(221), plt.imshow(image1, "gray"), plt.title("red line")
        # plt.subplot(222), plt.imshow(image2, "gray"), plt.title("green line")
        # plt.subplot(223), plt.plot(hist1, color="r"), plt.plot(hist2, color="g")
        # plt.xlim([0, 256])
        # plt.show()

        # publish each image to its corresponding topic
        for i, image in enumerate(images):
            # convert the OpenCV image to a ROS image message
            ros_img = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")

            # publish the ROS image message to thae corresponding topic
            self.pubs[i].publish(ros_img)

            # wait for the next publishing cycle
            self.rate.sleep()


# create an instance of the Image_Transformation_Traslation class
if __name__ == "__main__":
    image_histogram = Image_Histogram()
    try:
        # run the main function of the Image_Transformation_Traslation class until the ROS node is shut down
        while not rospy.is_shutdown():
            image_histogram.main()

    except rospy.ROSInterruptException:
        pass
