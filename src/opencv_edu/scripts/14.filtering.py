#!/usr/bin/env python3

import rospy
import rospkg
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np


class CvtColor:
    def __init__(self):
        rospy.init_node("image_node", anonymous=False)
        self.image_cvt_pub = rospy.Publisher("filter_img", Image, queue_size=10)
        self.rate = rospy.Rate(10)  # 10Hz

        rospack = rospkg.RosPack()
        self.file_path = rospack.get_path("opencv_edu")
        self.file_path += "/scripts/"

        self.lenna_path = self.file_path + "Lenna.png"
        self.brain_path = self.file_path + "brain.png"

    def main(self):
        self.lenna = cv2.imread(self.lenna_path, cv2.IMREAD_COLOR)
        self.brain = cv2.imread(self.brain_path, cv2.IMREAD_COLOR)
        self.kernel = np.ones((5, 5), np.float32) / 25

        # normal filtering, -1은 입력 영상과 동일한 데이터의 출력 영상 생성
        self.f2d_lenna = cv2.filter2D(self.lenna, -1, self.kernel)
        self.f2d_brain = cv2.filter2D(self.brain, -1, self.kernel)

        # gaussian filtering
        self.gaussian_lenna = cv2.GaussianBlur(self.lenna, (5, 5), 0)
        self.gaussian_brain = cv2.GaussianBlur(self.brain, (5, 5), 0)

        # medain filtering
        self.median_lenna = cv2.medianBlur(self.lenna, 5)
        self.median_brain = cv2.medianBlur(self.brain, 5)

        # bilateral filtering
        self.bilateral_lenna = cv2.bilateralFilter(self.lenna, 15, 40, 100)
        self.bilateral_brain = cv2.bilateralFilter(self.brain, 15, 40, 100)

        # compare filtering
        self.compare_lenna = cv2.hconcat(
            [
                self.lenna,
                self.f2d_lenna,
                self.gaussian_lenna,
                self.median_lenna,
                self.bilateral_lenna,
            ]
        )
        self.resize_lenna = cv2.resize(self.compare_lenna, [1000, 200])
        self.compare_brain = cv2.hconcat(
            [
                self.brain,
                self.f2d_brain,
                self.gaussian_brain,
                self.median_brain,
                self.bilateral_brain,
            ]
        )
        self.resize_brain = cv2.resize(self.compare_brain, [1000, 200])
        self.rst = cv2.vconcat([self.resize_lenna, self.resize_brain])

        # Initialize bridge to convert between ROS and OpenCV images
        self.bridge = CvBridge()
        image_msg = self.bridge.cv2_to_imgmsg(self.rst, "bgr8")
        self.image_cvt_pub.publish(image_msg)
        self.rate.sleep()


if __name__ == "__main__":
    cvtColor = CvtColor()
    try:
        while not rospy.is_shutdown():
            cvtColor.main()
    except rospy.ROSInterruptException:
        pass
