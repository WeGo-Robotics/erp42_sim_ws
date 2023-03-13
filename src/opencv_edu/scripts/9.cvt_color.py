#!/usr/bin/env python3

import rospy
import rospkg
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

# Define a class called CvtColor
class CvtColor:
    # Constructor method
    def __init__(self):
        # Initialize the node and the Publisher and Rate settings
        rospy.init_node("image_node", anonymous=False)
        # Create a publisher to publish the image to the "cvt_img" topic
        self.image_cvt_pub = rospy.Publisher("cvt_img", Image, queue_size=10)
        self.rate = rospy.Rate(10)  # 10Hz

        # Set the image path
        rospack = rospkg.RosPack()
        self.file_path = rospack.get_path("opencv_edu")
        self.file_path += "/scripts/"

        # Set the image file name
        self.wego_path = self.file_path + "wego.png"

    # Main method
    def main(self):
        # Load the image
        self.color_img = cv2.imread(self.wego_path, cv2.IMREAD_COLOR)
        self.gray_img = cv2.imread(self.wego_path, cv2.IMREAD_GRAYSCALE)

        # Convert the image
        self.cvt_gray = cv2.cvtColor(self.color_img, cv2.COLOR_BGR2GRAY)
        self.cvt_hsv = cv2.cvtColor(self.color_img, cv2.COLOR_BGR2HSV)
        self.cvt_hsl = cv2.cvtColor(self.color_img, cv2.COLOR_BGR2HLS)
        self.cvt_lab = cv2.cvtColor(self.color_img, cv2.COLOR_BGR2Lab)

        # Combine the images
        self.gray_3d = np.dstack((self.cvt_gray, self.cvt_gray, self.cvt_gray))
        self.cvt1 = cv2.hconcat([self.gray_3d, self.cvt_lab])
        self.cvt2 = cv2.hconcat([self.cvt_hsv, self.cvt_hsl])
        self.rst = cv2.vconcat([self.cvt1, self.cvt2])
        self.dst = cv2.resize(self.rst, (2000, 1000), interpolation=cv2.INTER_LINEAR)

        # Initialize bridge to convert between ROS and OpenCV images
        self.bridge = CvBridge()
        image_msg = self.bridge.cv2_to_imgmsg(self.dst, "bgr8")
        # Publish the image message
        self.image_cvt_pub.publish(image_msg)
        self.rate.sleep()


if __name__ == "__main__":
    cvtColor = CvtColor()
    try:
        while not rospy.is_shutdown():
            cvtColor.main()
    except rospy.ROSInterruptException:
        pass
