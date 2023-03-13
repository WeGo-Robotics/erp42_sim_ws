#!/usr/bin/env python3

import rospy
import rospkg
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np


class HSV_trackbar:
    def __init__(self):
        rospy.init_node("image_node", anonymous=False)
        self.image_cvt_pub = rospy.Publisher("cvt_img", Image, queue_size=10)
        self.rate = rospy.Rate(10)  # 10Hz

        rospack = rospkg.RosPack()
        self.file_path = rospack.get_path("opencv_edu")
        self.file_path += "/scripts/"

        self.hsv_path = self.file_path + "HSV.png"
        self.win_name = "color_detect_hsv"
        self.create_trackbar_init()

    def create_trackbar_init(self):
        # Trackbar를 표시할 윈도우 이름을 명시합니다.
        cv2.namedWindow(self.win_name, cv2.WINDOW_NORMAL)
        # Trackbar의 변수명, 윈도우 이름과 변수 범위를 설정합니다.
        cv2.createTrackbar("LH", self.win_name, 0, 179, self.hsv_track)
        cv2.createTrackbar("LS", self.win_name, 0, 255, self.hsv_track)
        cv2.createTrackbar("LV", self.win_name, 0, 255, self.hsv_track)
        cv2.createTrackbar("UH", self.win_name, 179, 179, self.hsv_track)
        cv2.createTrackbar("US", self.win_name, 255, 255, self.hsv_track)
        cv2.createTrackbar("UV", self.win_name, 255, 255, self.hsv_track)

    def hsv_track(self, value):
        # Trackbar의 조절한 값을 변수에 저장합니다.
        self.L_H_Value = cv2.getTrackbarPos("LH", self.win_name)
        self.L_S_Value = cv2.getTrackbarPos("LS", self.win_name)
        self.L_V_Value = cv2.getTrackbarPos("LV", self.win_name)
        self.U_H_Value = cv2.getTrackbarPos("UH", self.win_name)
        self.U_S_Value = cv2.getTrackbarPos("US", self.win_name)
        self.U_V_Value = cv2.getTrackbarPos("UV", self.win_name)

    def main(self):
        self.color_img = cv2.imread(self.hsv_path, cv2.IMREAD_COLOR)
        self.cvt_hsv = cv2.cvtColor(self.color_img, cv2.COLOR_BGR2HSV)
        self.lower = np.array([self.L_H_Value, self.L_S_Value, self.L_V_Value])
        self.upper = np.array([self.U_H_Value, self.U_S_Value, self.U_V_Value])
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
    hsv_trackbar = HSV_trackbar()
    try:
        while not rospy.is_shutdown():
            hsv_trackbar.main()
    except rospy.ROSInterruptException:
        pass
