#!/usr/bin/env python3

import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class Webcam:
    def __init__(self):
        rospy.init_node("webcam_node", anonymous=True)
        self.image_pub = rospy.Publisher("webcam_img", Image, queue_size=1)

        self.capture = cv2.VideoCapture(0)
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.capture.set(cv2.CAP_PROP_FPS, 30)

    def main(self):
        self.bridge = CvBridge()
        _, self.img = self.capture.read()
        webcam_img_msg = self.bridge.cv2_to_imgmsg(self.img, "bgr8")
        self.image_pub.publish(webcam_img_msg)


if __name__ == "__main__":
    webcam = Webcam()
    try:
        while not rospy.is_shutdown():
            webcam.main()
    except rospy.ROSInterruptException:
        pass
