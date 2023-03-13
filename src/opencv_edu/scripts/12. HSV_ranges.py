#!/usr/bin/env python3

import rospy
import rospkg
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np


class HSV_In_Range:
    def __init__(self):
        rospy.init_node("image_node", anonymous=False)
        self.image_cvt_pub = rospy.Publisher("cvt_img", Image, queue_size=10)
        self.rate = rospy.Rate(10)  # 10Hz

        rospack = rospkg.RosPack()
        self.file_path = rospack.get_path("opencv_edu")
        self.file_path += "/scripts/"

        self.hsv_path = self.file_path + "HSV.png"

    def main(self):
        self.color_img = cv2.imread(self.hsv_path, cv2.IMREAD_COLOR)
        self.cvt_hsv = cv2.cvtColor(self.color_img, cv2.COLOR_BGR2HSV)

        self.lower = [0, 0, 0]
        self.upper = [179, 255, 255]

        self.mask = cv2.inRange(self.cvt_hsv, self.lower, self.upper)
        self.rst = cv2.bitwise_and(self.color_img, self.color_img, mask=self.mask)
        # np array를 string으로 변경합니다. (디스플레이용)
        lower_string = "lower H,S,V : " + ",".join(str(e) for e in self.lower.tolist())
        upper_string = "upper H,S,V : " + ",".join(str(e) for e in self.upper.tolist())
        cv2.putText(
            self.rst,
            lower_string,
            (25, 75),
            cv2.FONT_HERSHEY_SIMPLEX,
            2,
            (255, 255, 255),
            3,
        )
        cv2.putText(
            self.rst,
            upper_string,
            (25, 150),
            cv2.FONT_HERSHEY_SIMPLEX,
            2,
            (255, 255, 255),
            3,
        )
        cv2.imshow(self.win_name, self.rst)
        cv2.waitKey(1)
        # Initialize bridge to convert between ROS and OpenCV images
        self.bridge = CvBridge()
        image_msg = self.bridge.cv2_to_imgmsg(self.rst, "bgr8")
        self.image_cvt_pub.publish(image_msg)
        self.rate.sleep()


if __name__ == "__main__":
    hsv_in_range = HSV_In_Range()
    try:
        while not rospy.is_shutdown():
            hsv_in_range.main()
    except rospy.ROSInterruptException:
        pass
