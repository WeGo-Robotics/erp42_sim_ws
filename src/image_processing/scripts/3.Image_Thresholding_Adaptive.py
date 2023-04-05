#!/usr/bin/env python3

# import libraries
import cv2  # OpenCV library for image processing
import rospy  # ROS Python library for working with ROS
from sensor_msgs.msg import Image  # ROS Image message type
from cv_bridge import CvBridge  # Library to convert between ROS and OpenCV images
import rospkg  # ROS package library


class ImageThreshold_Adaptive:
    # constructor function, takes no input arguments
    def __init__(self):
        # initialize a ROS node
        rospy.init_node("image_node")
        # set the publishing rate
        self.rate = rospy.Rate(10)
        # Get the file path of the "image_processing" package
        rospack = rospkg.RosPack()
        self.file_path = rospack.get_path("image_processing")
        self.file_path += "/scripts/"

        # Set the file path for the image file
        self.image_path = self.file_path + "sudoku.png"

        # Initialize bridge to convert between ROS and OpenCV images
        self.bridge = CvBridge()

        # Create publishers for each image
        self.pubs = []
        titles = ["Original", "Global", "Mean", "Gaussian"]
        for title in titles:
            pub = rospy.Publisher("/image_topic/" + title, Image, queue_size=1)
            self.pubs.append(pub)

    # Load image, apply thresholding methods, and publish them as ROS images
    def main(self):
        # Load image
        image = cv2.imread(self.image_path, 0)

        # Apply different thresholding methods
        th1 = cv2.threshold(image, 127, 255, cv2.THRESH_BINARY)[1]
        th2 = cv2.adaptiveThreshold(
            image, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 15, 2
        )
        th3 = cv2.adaptiveThreshold(
            image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 15, 2
        )

        # Store images in a list
        images = [image, th1, th2, th3]

        # Publish each image as a ROS image message
        for i, image in enumerate(images):
            ros_img = self.bridge.cv2_to_imgmsg(image, encoding="passthrough")
            self.pubs[i].publish(ros_img)


if __name__ == "__main__":
    # create an instance of the ImageThreshold class
    image_threshold_adaptive = ImageThreshold_Adaptive()
    try:
        # run the main function of the ImageThreshold class until the ROS node is shut down
        while not rospy.is_shutdown():
            image_threshold_adaptive.main()
    except rospy.ROSInterruptException:
        # catch any exceptions that may occur during the ROS node execution
        pass
