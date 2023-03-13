#!/usr/bin/env python3

import rospy
import rospkg
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np


class HSV_In_Range:
    def __init__(self):
        # Initialize the ROS node with name "image_node"
        rospy.init_node("image_node", anonymous=False)
        # Create a publisher that publishes messages of type Image to the topic "cvt_img"
        self.image_cvt_pub = rospy.Publisher("cvt_img", Image, queue_size=10)
        self.rate = rospy.Rate(10)  # 10Hz

        rospack = rospkg.RosPack()
        self.file_path = rospack.get_path("opencv_edu")
        self.file_path += "/scripts/"

        # Get the file path for the opencv_edu package and the location of the HSV.png file
        self.hsv_path = self.file_path + "HSV.png"

    def main(self):
        # Read the HSV.png file and convert it to the HSV color space
        self.color_img = cv2.imread(self.hsv_path, cv2.IMREAD_COLOR)
        self.cvt_hsv = cv2.cvtColor(self.color_img, cv2.COLOR_BGR2HSV)

        # Set the lower and upper bounds for the color range in the HSV color space
        self.lower = [0, 0, 0]
        self.upper = [179, 255, 255]

        # Create a mask that extracts the pixels in the specified color range
        self.mask = cv2.inRange(self.cvt_hsv, self.lower, self.upper)
        # Apply the mask to the original color image to obtain the masked image
        self.rst = cv2.bitwise_and(self.color_img, self.color_img, mask=self.mask)
        # Convert the lower and upper bounds to strings for display purposes
        lower_string = "lower H,S,V : " + ",".join(str(e) for e in self.lower.tolist())
        upper_string = "upper H,S,V : " + ",".join(str(e) for e in self.upper.tolist())
        # Add text to the masked image indicating the lower and upper bounds of the color range
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
        # Show the masked image in a window named "result"
        cv2.imshow(self.win_name, self.rst)
        cv2.waitKey(1)
        # Initialize bridge to convert between ROS and OpenCV images
        self.bridge = CvBridge()
        # Convert the masked image to an Image message and publish it to the "cvt_img" topic
        image_msg = self.bridge.cv2_to_imgmsg(self.rst, "bgr8")
        self.image_cvt_pub.publish(image_msg)
        # Sleep to maintain the rate of the node
        self.rate.sleep()


if __name__ == "__main__":
    hsv_in_range = HSV_In_Range()
    try:
        while not rospy.is_shutdown():
            hsv_in_range.main()
    except rospy.ROSInterruptException:
        pass
