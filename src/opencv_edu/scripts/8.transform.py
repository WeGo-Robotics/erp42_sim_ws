#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

# Define a class named "Transform"
class Transform:
    # Define the constructor of the class
    def __init__(self):
        # Initialize a ROS node named "image_node"
        rospy.init_node("image_node", anonymous=False)
        # Create two publishers for two image topics named "affine_img" and "perspective_img"
        self.affine_pub = rospy.Publisher("affine_img", Image, queue_size=10)
        self.perspective_pub = rospy.Publisher("perspective_img", Image, queue_size=10)
        self.rate = rospy.Rate(10)  # 10Hz
        
        # Define color variables for blue, green, red, yellow, and white
        self.blue = [255, 0, 0]
        self.green = [0, 255, 0]
        self.red = [0, 0, 255]
        self.yellow = [0, 255, 255]
        self.white = [255, 255, 255]
        # Define the shape of the image as [1000, 1000]
        self.shape = [1000, 1000]
        # Set the height and width variables based on the shape of the image
        self.height, self.width = self.shape

        # Define an 8x10 2D numpy array representing a chessboard patternㄱ
        self.chessboard = np.uint8(
            [
                [0, 255, 0, 255, 0, 255, 0, 255, 0, 255],
                [255, 0, 255, 0, 255, 0, 255, 0, 255, 0],
                [0, 255, 0, 255, 0, 255, 0, 255, 0, 255],
                [255, 0, 255, 0, 255, 0, 255, 0, 255, 0],
                [0, 255, 0, 255, 0, 255, 0, 255, 0, 255],
                [255, 0, 255, 0, 255, 0, 255, 0, 255, 0],
                [0, 255, 0, 255, 0, 255, 0, 255, 0, 255],
                [255, 0, 255, 0, 255, 0, 255, 0, 255, 0],
                [0, 255, 0, 255, 0, 255, 0, 255, 0, 255],
                [255, 0, 255, 0, 255, 0, 255, 0, 255, 0],
            ]
        )

        # Define a 100x100 2D numpy array with two diagonal white lines
        self.two_lines = np.zeros((100, 100), np.uint8)
        cv2.line(self.two_lines, (10, 90), (40, 50), self.white, 2)
        cv2.line(self.two_lines, (90, 90), (60, 50), self.white, 2)

        # Resize the chessboard image to the specified shape, binarize it, and stack it with 3 channels
        self.resize_cb = cv2.resize(self.chessboard, self.shape)
        self.resize_cb[self.resize_cb <= 127] = 0
        self.resize_cb[self.resize_cb > 127] = 255
        self.resize_cb_3d = np.dstack((self.resize_cb, self.resize_cb, self.resize_cb))

        # Resize the "two_lines" array to the specified shape, binarize it, and stack it with 3 channels
        self.resize_tw = cv2.resize(self.two_lines, self.shape)
        self.resize_tw[self.resize_tw <= 127] = 0
        self.resize_tw[self.resize_tw > 127] = 255
        self.resize_tw_3d = np.dstack((self.resize_tw, self.resize_tw, self.resize_tw))

    def main(self):
        self.bridge = CvBridge()

        # Define 3 points on the chessboard image and draw circles on them in blue, green, and red colors
        self.a_point1 = [100, 100]
        self.a_point2 = [100, 200]
        self.a_point3 = [200, 100]

        # Draw a blue circle with radius 10 pixels on point1
        cv2.circle(self.resize_cb_3d, self.a_point1, 10, self.blue, -1)
        # Draw a green circle with radius 10 pixels on point2
        cv2.circle(self.resize_cb_3d, self.a_point2, 10, self.green, -1)
        # Draw a red circle with radius 10 pixels on point3
        cv2.circle(self.resize_cb_3d, self.a_point3, 10, self.red, -1)

        # Define 4 points on the "two_lines" array and draw circles on them in blue, green, red, and yellow colors
        self.p_point1 = [100, 900]
        self.p_point2 = [400, 500]
        self.p_point3 = [600, 500]
        self.p_point4 = [900, 900]

        # Draw a blue circle with radius 20 pixels on point1
        cv2.circle(self.resize_tw_3d, self.p_point1, 20, self.blue, -1)
        # Draw a green circle with radius 20 pixels on point2
        cv2.circle(self.resize_tw_3d, self.p_point2, 20, self.green, -1)
        # Draw a red circle with radius 20 pixels on point3
        cv2.circle(self.resize_tw_3d, self.p_point3, 20, self.red, -1)
        # Draw a yellow circle with radius 20 pixels on point4
        cv2.circle(self.resize_tw_3d, self.p_point4, 20, self.yellow, -1)

        self.a_src_point = np.float32(
            [self.a_point1, self.a_point2, self.a_point3],
        )

        self.a_dst_point = np.float32(
            [
                [self.a_point1[0], self.a_point1[1]],
                [self.a_point2[0] * 2, self.a_point2[1] * 2],
                [self.a_point3[0] * 2, self.a_point3[1] * 2],
            ]
        )

        # Define four source points for perspective transformation
        self.p_src_point = np.float32(
            [self.p_point1, self.p_point2, self.p_point3, self.p_point4]
        )
        # Define four destination points for perspective transformation
        self.p_dst_point = np.float32([[100, 1000], [100, 0], [900, 0], [900, 1000]])

        
# Get affine transformation matrix and its inverse
        self.a_matrix = cv2.getAffineTransform(self.a_src_point, self.a_dst_point)
        self.a_matrix_inv = cv2.getAffineTransform(self.a_dst_point, self.a_src_point)

        # Get perspective transformation matrix and its inverse
        self.p_matrix = cv2.getPerspectiveTransform(self.p_src_point, self.p_dst_point)
        self.p_matrix_inv = cv2.getPerspectiveTransform(
            self.p_dst_point, self.p_src_point
        )

        # Apply affine and perspective transformations to images
        self.a_cb_3d = cv2.warpAffine(
            self.resize_cb_3d, self.a_matrix, (self.width, self.height)
        )
        self.p_tw_3d = cv2.warpPerspective(
            self.resize_tw_3d, self.p_matrix, (self.width, self.height)
        )

        # Concatenate original and transformed chessboard images horizontally with lines to indicate points
        self.a_dst = cv2.hconcat([self.resize_cb_3d, self.a_cb_3d])
        cv2.line(
            self.a_dst,
            self.a_point1,
            (self.a_point1[0] + self.width, self.a_point1[1]),
            self.blue,
            10,
        )
        cv2.line(
            self.a_dst,
            self.a_point2,
            (self.a_point2[0] * 2 + self.width, self.a_point2[1] * 2),
            self.green,
            10,
        )
        cv2.line(
            self.a_dst,
            self.a_point3,
            (self.a_point3[0] * 2 + self.width, self.a_point3[1] * 2),
            self.red,
            10,
        )

        # Concatenate original and transformed two_lines images horizontally with lines to indicate points
        self.p_dst = cv2.hconcat([self.resize_tw_3d, self.p_tw_3d])
        cv2.line(
            self.p_dst,
            self.p_point1,
            ([100 + self.width, 1000]),
            self.blue,
            10,
        )
        cv2.line(
            self.p_dst,
            self.p_point2,
            ([100 + self.width, 0]),
            self.green,
            10,
        )
        cv2.line(
            self.p_dst,
            self.p_point3,
            ([900 + self.width, 0]),
            self.red,
            10,
        )
        cv2.line(
            self.p_dst,
            self.p_point4,
            ([900 + self.width, 1000]),
            self.yellow,
            10,
        )

        # Convert images to ROS message and publish them
        a_image_msg = self.bridge.cv2_to_imgmsg(self.a_dst, "bgr8")
        p_image_msg = self.bridge.cv2_to_imgmsg(self.p_dst, "bgr8")
        self.affine_pub.publish(a_image_msg)
        self.perspective_pub.publish(p_image_msg)
        # Sleep to maintain the rate of the node
        self.rate.sleep()


if __name__ == "__main__":
    transform = Transform()
    try:
        while not rospy.is_shutdown():
            transform.main()
    except rospy.ROSInterruptException:
        pass
