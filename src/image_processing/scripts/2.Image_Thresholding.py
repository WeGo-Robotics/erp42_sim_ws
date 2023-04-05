#!/usr/bin/env python3

# import libraries
import cv2  # OpenCV library for image processing
import rospy  # ROS Python library for working with ROS
from sensor_msgs.msg import Image  # ROS Image message type
from cv_bridge import CvBridge  # Library to convert between ROS and OpenCV images
import rospkg  # ROS package library


# define a class called ImageThreshold
class Image_Threshold:
    # constructor function, takes no input arguments
    def __init__(self):
        # initialize a ROS node
        rospy.init_node("image_node")
        # define a list of titles for the images to be displayed
        titles = ["Original", "BINARY", "BINARY_INV", "TRUNC", "TOZERO", "TOZERO_INV"]
        # set the publishing rate
        self.rate = rospy.Rate(10)
        # Get the file path of the "image_processing" package
        rospack = rospkg.RosPack()
        self.file_path = rospack.get_path("image_processing")
        self.file_path += "/scripts/"

        # Set the file path for the image file
        self.image_path = self.file_path + "gradient.png"

        # Initialize bridge to convert between ROS and OpenCV images
        self.bridge = CvBridge()

        self.pubs = []
        for title in titles:
            pub = rospy.Publisher("/image_topic/" + title, Image, queue_size=1)
            self.pubs.append(pub)

    # publish the images to ROS topics
    def main(self):
        # load the image file as grayscale
        img = cv2.imread(self.image_path)

        # apply different thresholding techniques on the image and store the results in a list
        thresholds = [
            cv2.threshold(img, 127, 255, cv2.THRESH_BINARY),
            cv2.threshold(img, 127, 255, cv2.THRESH_BINARY_INV),
            cv2.threshold(img, 127, 255, cv2.THRESH_TRUNC),
            cv2.threshold(img, 127, 255, cv2.THRESH_TOZERO),
            cv2.threshold(img, 127, 255, cv2.THRESH_TOZERO_INV),
        ]

        # create a list of all the images including the original and the thresholded ones
        images = [img] + [x[1] for x in thresholds]
        # create a list of publishers, one for each image

        # publish each image to its corresponding topic
        for i, image in enumerate(images):
            ros_img = self.bridge.cv2_to_imgmsg(image, encoding="rgb8")
            self.pubs[i].publish(ros_img)
            # wait for the next publishing cycle
            self.rate.sleep()


if __name__ == "__main__":
    # create an instance of the ImageThreshold class
    image_threshold = Image_Threshold()
    try:
        # run the main function of the ImageThreshold class until the ROS node is shut down
        while not rospy.is_shutdown():
            image_threshold.main()
    except rospy.ROSInterruptException:
        # catch any exceptions that may occur during the ROS node execution
        pass
