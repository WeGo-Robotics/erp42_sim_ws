#!/usr/bin/env python3

import rospy
import rospkg
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np


class Bitwise_Operation:
    def __init__(self):
        rospy.init_node("image_node", anonymous=False)
        self.image_pub = rospy.Publisher("bitwise_img", Image, queue_size=10)
        self.rate = rospy.Rate(10)  # 10Hz

        rospack = rospkg.RosPack()

        self.file_path = rospack.get_path("opencv_edu")
        self.file_path += "/scripts/"

        self.lenna_path = self.file_path + "Lenna.png"
        self.wego_path = self.file_path + "wego.png"

    def main(self):
        # Initialize bridge to convert between ROS and OpenCV images
        bridge = CvBridge()
        self.lenna = cv2.imread(self.lenna_path, cv2.IMREAD_COLOR)
        self.wego = cv2.imread(self.wego_path, cv2.IMREAD_COLOR)
        lenna_gray = cv2.cvtColor(self.lenna, cv2.COLOR_BGR2GRAY)
        wego_gray = cv2.cvtColor(self.wego, cv2.COLOR_BGR2GRAY)
        _, wego_binary = cv2.threshold(wego_gray, 127, 255, cv2.THRESH_BINARY)
        _, wego_binary_inv = cv2.threshold(wego_gray, 127, 255, cv2.THRESH_BINARY_INV)
        resize_img = cv2.resize(wego_binary, (lenna_gray.shape[1], lenna_gray.shape[0]))
        resize_img_inv = cv2.resize(
            wego_binary_inv, (lenna_gray.shape[1], lenna_gray.shape[0])
        )

        and_img = cv2.bitwise_and(lenna_gray, resize_img)
        or_img = cv2.bitwise_or(lenna_gray, resize_img)
        xor_img = cv2.bitwise_xor(lenna_gray, resize_img)
        not_img = cv2.bitwise_not(resize_img)

        and_img_inv = cv2.bitwise_and(lenna_gray, resize_img_inv)
        or_img_inv = cv2.bitwise_or(lenna_gray, resize_img_inv)
        xor_img_inv = cv2.bitwise_xor(lenna_gray, resize_img_inv)
        not_img_inv = cv2.bitwise_not(resize_img_inv)

        dst1 = cv2.hconcat([and_img, and_img_inv, or_img, or_img_inv])
        dst2 = cv2.hconcat([xor_img, xor_img_inv, not_img, not_img_inv])

        rst = cv2.vconcat([dst1, dst2])
        img_msg = bridge.cv2_to_imgmsg(rst)
        self.image_pub.publish(img_msg)
        self.rate.sleep()

        cv2.destroyAllWindows()


if __name__ == "__main__":
    bitwise_operation = Bitwise_Operation()
    try:
        while not rospy.is_shutdown():
            bitwise_operation.main()

    except rospy.ROSInterruptException:
        pass
