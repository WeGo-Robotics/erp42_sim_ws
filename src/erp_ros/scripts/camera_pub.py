#!/usr/bin/env python3
  
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import  CompressedImage
from cv_bridge import CvBridge, CvBridgeError

class IMGParser:
    def __init__(self):
        rospy.init_node('camera', anonymous=True)
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        self.image_pub = rospy.Publisher("/image_jpeg/gray/compressed",CompressedImage,queue_size=10)
        self.bridge = CvBridge()
        rospy.spin()

    def callback(self, data):
        img_bgr = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        img_gray = cv2.cvtColor(img_bgr,cv2.COLOR_BGR2GRAY)
        cv2.imshow("img_bgr", img_bgr)
        cv2.imshow("img_gray", img_gray)
        self.msg = CompressedImage()
        self.msg.header.stamp = rospy.Time.now()
        self.msg.format = "jpeg"
        self.msg.data = self.bridge.cv2_to_compressed_imgmsg(img_gray, "jpeg").data
        self.image_pub.publish(self.msg)
        cv2.waitKey(1)
      

if __name__ == '__main__':
    try:
        image_parser = IMGParser()
    except rospy.ROSInterruptException:
        pass