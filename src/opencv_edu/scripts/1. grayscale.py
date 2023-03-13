#!/usr/bin/env python3

import rospy
import rospkg
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np


class Gray_Scale:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("image_node", anonymous=False)
        # Create a publisher to publish the image
        self.image_pub = rospy.Publisher("gray_img", Image, queue_size=10)
        self.rate = rospy.Rate(10)
        # Define image to be published
        self.gray = np.array(
            [
                [0, 255, 127, 255, 0],
                [255, 127, 255, 127, 255],
                [127, 255, 0, 255, 127],
                [255, 127, 255, 127, 255],
                [0, 255, 127, 255, 0],
            ],
            np.uint8,
        )

    def main(self):
        # Initialize bridge to convert between ROS and OpenCV images
        self.bridge = CvBridge()

        # Convert the OpenCV image to a ROS image message
        self.ros_image = self.bridge.cv2_to_imgmsg(self.gray)

        # Publish the ROS image message
        self.image_pub.publish(self.ros_image)
        self.rate.sleep()


if __name__ == "__main__":
    gray_scale = Gray_Scale()
    try:
        while not rospy.is_shutdown():
            gray_scale.main()
    except rospy.ROSInterruptException:
        pass
