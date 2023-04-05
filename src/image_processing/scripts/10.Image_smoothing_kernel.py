#!/usr/bin/env python3

# import libraries
import cv2  # OpenCV library for image processing
import numpy as np  # NumPy library for numerical operations
import rospy  # ROS Python library for working with ROS
from sensor_msgs.msg import Image  # ROS Image message type
from cv_bridge import CvBridge  # Library to convert between ROS and OpenCV images
import rospkg  # ROS package library


class Image_smoothing_kernel:
    def __init__(self):
        # initialize a ROS node
        rospy.init_node("image_node")

        # define a list of titles for the images to be displayed
        self.titles = ["kernel"]

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
            # create a publisher for each title in the titles list
            pub = rospy.Publisher("/image_topic/" + title, Image, queue_size=1)
            # append the publisher to the pubs list
            self.pubs.append(pub)

    # Define the callback function for the trackbar
    def nothing(x):
        pass

    # Define the main function for image filtering
    def main(self):
        # Load image from file
        img = cv2.imread(self.image_path)
        # Create a window named 'image' to display the filtered image
        cv2.namedWindow("image")

        # Create a trackbar named 'K' to adjust the kernel size
        cv2.createTrackbar("K", "image", 1, 20, self.nothing)

        while True:
            # Wait for trackbar to change value
            k = cv2.getTrackbarPos("K", "image")
            if k == 0:
                k = 1

            # Apply filter to original image
            kernel = np.ones((k, k), np.float32) / (
                k**2
            )  # define a kernel of size k x k
            dst = cv2.filter2D(img, -1, kernel)  # apply 2D filter to image

            # Publish filtered image as ROS image message
            filtered_img_msg = self.bridge.cv2_to_imgmsg(dst, encoding="bgr8")
            self.pubs[0].publish(filtered_img_msg)

            # Display original image with filter
            cv2.imshow("image", dst)

            # Exit the while loop when the ESC key is pressed
            if cv2.waitKey(1) & 0xFF == 27:
                break

        cv2.destroyAllWindows()


# create an instance of the Image_Transformation_Traslation class
if __name__ == "__main__":
    image_smoothing_kernel = Image_smoothing_kernel()
    try:
        # run the main function of the Image_Transformation_Traslation class until the ROS node is shut down
        while not rospy.is_shutdown():
            image_smoothing_kernel.main()

    except rospy.ROSInterruptException:
        pass
