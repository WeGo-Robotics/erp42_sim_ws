#!/usr/bin/env python3

import rospy
import rospkg
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np


class Filter:
    # Define the initialization function of the class
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("image_node", anonymous=False)

        # Create a publisher to publish the image
        self.image_filter_pub = rospy.Publisher("filter_img", Image, queue_size=10)

        # Set the publishing rate to 10
        self.rate = rospy.Rate(10)

        # Get the file path of the "opencv_edu" package
        rospack = rospkg.RosPack()
        self.file_path = rospack.get_path("opencv_edu")
        self.file_path += "/scripts/"

        # Set the file path for the image files
        self.lenna_path = self.file_path + "Lenna.png"
        self.brain_path = self.file_path + "brain.png"

        # Initialize bridge to convert between ROS and OpenCV images
        self.bridge = CvBridge()

    # Define the main function of the class
    def main(self):
        # Read the images in BGR format using OpenCV
        lenna = cv2.imread(self.lenna_path, cv2.IMREAD_COLOR)
        brain = cv2.imread(self.brain_path, cv2.IMREAD_COLOR)
        kernel = np.ones((5, 5), np.float32) / 25

        # normal filtering, -1은 입력 영상과 동일한 데이터의 출력 영상 생성
        f2d_lenna = cv2.filter2D(lenna, -1, kernel)
        f2d_brain = cv2.filter2D(brain, -1, kernel)

        # gaussian filtering
        gaussian_lenna = cv2.GaussianBlur(lenna, (5, 5), 0)
        gaussian_brain = cv2.GaussianBlur(brain, (5, 5), 0)

        # medain filtering
        median_lenna = cv2.medianBlur(lenna, 5)
        median_brain = cv2.medianBlur(brain, 5)

        # bilateral filtering
        bilateral_lenna = cv2.bilateralFilter(lenna, 15, 40, 100)
        bilateral_brain = cv2.bilateralFilter(brain, 15, 40, 100)

        # compare filtering
        compare_lenna = cv2.hconcat(
            [
                lenna,
                f2d_lenna,
                gaussian_lenna,
                median_lenna,
                bilateral_lenna,
            ]
        )

        # Resize the comparison image to match the size of the Lenna image
        resize_lenna = cv2.resize(compare_lenna, [1000, 200])

        # Concatenate original and filtered lenna images horizontally
        compare_brain = cv2.hconcat(
            [
                brain,
                f2d_brain,
                gaussian_brain,
                median_brain,
                bilateral_brain,
            ]
        )

        # Resize the comparison image to match the size of the brain image
        resize_brain = cv2.resize(compare_brain, [1000, 200])

        # Concatenate the resized lenna image and the resized brain image vertically
        rst = cv2.vconcat([resize_lenna, resize_brain])

        # Convert the OpenCV image to a ROS image message
        image_msg = self.bridge.cv2_to_imgmsg(rst, "bgr8")

        # Publish the ROS image message
        self.image_filter_pub.publish(image_msg)

        # Sleep for a certain duration to maintain the publishing rate
        self.rate.sleep()


if __name__ == "__main__":
    filter = Filter()
    try:
        while not rospy.is_shutdown():
            filter.main()
    except rospy.ROSInterruptException:
        pass
