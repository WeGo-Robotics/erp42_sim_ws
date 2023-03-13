#!/usr/bin/env python3

import rospy
import rospkg
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np


class Binary:
    def __init__(self):
        rospy.init_node("image_node", anonymous=False)
        self.image_pub = rospy.Publisher("binary_img", Image, queue_size=10)
        self.rate = rospy.Rate(10)  # 10Hz

        rospack = rospkg.RosPack()
        self.file_path = rospack.get_path("opencv_edu")
        self.file_path += "/scripts/"
        self.wego_path = self.file_path + "wego.png"

    def main(self):
        # Initialize bridge to convert between ROS and OpenCV images
        bridge = CvBridge()
        self.wego = cv2.imread(self.wego_path, cv2.IMREAD_COLOR)
        wego_gray = cv2.cvtColor(self.wego, cv2.COLOR_BGR2GRAY)
        _, wego_binary = cv2.threshold(wego_gray, 127, 255, cv2.THRESH_BINARY)
        _, wego_binary_INV = cv2.threshold(wego_gray, 127, 255, cv2.THRESH_BINARY_INV)
        con_binary = cv2.hconcat((wego_binary, wego_binary_INV))
        resize_con_binary = cv2.resize(
            con_binary, (wego_gray.shape[1], wego_gray.shape[0])
        )
        thresh_rst = cv2.vconcat((wego_gray, resize_con_binary))
        image_msg = bridge.cv2_to_imgmsg(thresh_rst)
        self.image_pub.publish(image_msg)
        self.rate.sleep()


if __name__ == "__main__":
    binary = Binary()
    try:
        while not rospy.is_shutdown():
            binary.main()
    except rospy.ROSInterruptException:
        pass
