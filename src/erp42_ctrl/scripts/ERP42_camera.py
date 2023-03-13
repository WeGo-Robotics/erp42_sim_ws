#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import numpy as np
import cv2


class Camera:
    def __init__(self):
        self.bridge = CvBridge()

        rospy.init_node("camera_node")

        # Subscribe to Morai ImageData topic
        self.image_sub = rospy.Subscriber(
            "/image_jpeg/compressed", CompressedImage, self.camera_CB
        )
        # Advertise ROS Image topic
        self.image_pub = rospy.Publisher(
            "/camera/compressed", CompressedImage, queue_size=10
        )

    def camera_CB(self, msg):
        # Convert CompressedImage to numpy array
        self.sub_img_msg = CompressedImage()
        self.sub_img_msg = msg
        self.img = self.bridge.compressed_imgmsg_to_cv2(self.sub_img_msg)

        # Publish cv2 image as ROS Image message
        self.pub_img_msg = self.bridge.cv2_to_compressed_imgmsg(self.img)
        self.image_pub.publish(self.pub_img_msg)


if __name__ == "__main__":
    try:
        camera = Camera()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
