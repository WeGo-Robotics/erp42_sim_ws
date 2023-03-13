#!/usr/bin/env python3

import rospy
import rospkg
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np


class HSV_trackbar:
    def __init__(self):
        # Initialize the ROS node with a unique name
        rospy.init_node("image_node", anonymous=False)
        # Publisher for the converted image
        self.image_cvt_pub = rospy.Publisher("cvt_img", Image, queue_size=10)
        self.rate = rospy.Rate(10)  # 10Hz

        # Create an instance of the rospack object
        rospack = rospkg.RosPack()
        # Get the path to the package containing the script
        self.file_path = rospack.get_path("opencv_edu")
        # Append the script directory to the file path
        self.file_path += "/scripts/"

        # The path to the HSV image file
        self.hsv_path = self.file_path + "HSV.png"
        # The name of the window for the trackbar GUI
        self.win_name = "color_detect_hsv"
        # Create the trackbars for the HSV ranges
        self.create_trackbar_init()

    def create_trackbar_init(self):
        # Create a named window for the trackbar GUI
        cv2.namedWindow(self.win_name, cv2.WINDOW_NORMAL)
        # Create trackbars for the HSV range
        cv2.createTrackbar("LH", self.win_name, 0, 179, self.hsv_track)
        cv2.createTrackbar("LS", self.win_name, 0, 255, self.hsv_track)
        cv2.createTrackbar("LV", self.win_name, 0, 255, self.hsv_track)
        cv2.createTrackbar("UH", self.win_name, 179, 179, self.hsv_track)
        cv2.createTrackbar("US", self.win_name, 255, 255, self.hsv_track)
        cv2.createTrackbar("UV", self.win_name, 255, 255, self.hsv_track)

    def hsv_track(self, value):
        # Get the values of the trackbars
        self.L_H_Value = cv2.getTrackbarPos("LH", self.win_name)
        self.L_S_Value = cv2.getTrackbarPos("LS", self.win_name)
        self.L_V_Value = cv2.getTrackbarPos("LV", self.win_name)
        self.U_H_Value = cv2.getTrackbarPos("UH", self.win_name)
        self.U_S_Value = cv2.getTrackbarPos("US", self.win_name)
        self.U_V_Value = cv2.getTrackbarPos("UV", self.win_name)

    def main(self):
        # Load the HSV image file
        self.color_img = cv2.imread(self.hsv_path, cv2.IMREAD_COLOR)
        # Convert the BGR image to HSV
        self.cvt_hsv = cv2.cvtColor(self.color_img, cv2.COLOR_BGR2HSV)
        # Define the HSV range using the trackbar values
        self.lower = np.array([self.L_H_Value, self.L_S_Value, self.L_V_Value])
        self.upper = np.array([self.U_H_Value, self.U_S_Value, self.U_V_Value])
        # Create a mask using the HSV range
        self.mask = cv2.inRange(self.cvt_hsv, self.lower, self.upper)
        # Apply the mask to the original image to obtain the final result
        self.rst = cv2.bitwise_and(self.color_img, self.color_img, mask=self.mask)
        # Convert np array to string for display purposes
        lower_string = "lower H,S,V : " + ",".join(str(e) for e in self.lower.tolist())
        upper_string = "upper H,S,V : " + ",".join(str(e) for e in self.upper.tolist())
        # Add text to the image
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
        # Display the image in a window
        cv2.imshow(self.win_name, self.rst)
        # Wait for a key press
        cv2.waitKey(1)
        # Initialize bridge to convert between ROS and OpenCV images
        self.bridge = CvBridge()
        # Convert the masked image to an Image message and publish it to the "cvt_img" topic
        image_msg = self.bridge.cv2_to_imgmsg(self.rst, "bgr8")
        self.image_cvt_pub.publish(image_msg)
        # Sleep to maintain the rate of the node
        self.rate.sleep()


if __name__ == "__main__":
    hsv_trackbar = HSV_trackbar()
    try:
        while not rospy.is_shutdown():
            hsv_trackbar.main()
    except rospy.ROSInterruptException:
        pass
