#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import numpy as np
import cv2


class Binary_Line:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node("binary_line_node")
        self.pub = rospy.Publisher("/binary/compressed", CompressedImage, queue_size=10)
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.img_CB)

    def detect_color(self, img):
        # Convert to HSV color space
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Define range of yellow color in HSV
        yellow_lower = np.array([15, 80, 0])
        yellow_upper = np.array([45, 255, 255])

        # Define range of blend color in HSV
        white_lower = np.array([0, 0, 200])
        white_upper = np.array([179, 64, 255])

        # Threshold the HSV image to get only yellow colors
        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)

        # Threshold the HSV image to get only white colors
        white_mask = cv2.inRange(hsv, white_lower, white_upper)

        # Threshold the HSV image to get blend colors
        blend_mask = cv2.bitwise_or(yellow_mask, white_mask)
        blend_color = cv2.bitwise_and(img, img, mask=blend_mask)
        return blend_color

    def img_warp(self, img, blend_color):
        # shape of img
        img_x, img_y = img.shape[1], img.shape[0]
        # print(f'img_x:{img_x}, img_y:{img_y}')
        # img_size = [640, 480]

        # ROI
        src_side_offset = [30, 100]
        src_center_offset = [100, 40]
        src = np.float32(
            [
                [src_side_offset[0], img_y - src_side_offset[1]],
                [img_x / 2 - src_center_offset[0], img_y / 2 + src_center_offset[1]],
                [img_x / 2 + src_center_offset[0], img_y / 2 + src_center_offset[1]],
                [img_x - src_side_offset[0], img_y - src_side_offset[1]],
            ]
        )
        # 아래 2 개 점 기준으로 dst 영역을 설정합니다.
        dst_offset = [80, 0]
        # offset x 값이 작아질 수록 dst box width 증가합니다.
        dst = np.float32(
            [
                [dst_offset[0], img_y],
                [dst_offset[0], 0],
                [img_x - dst_offset[0], 0],
                [img_x - dst_offset[0], img_y],
            ]
        )
        # find perspective matrix
        matrix = cv2.getPerspectiveTransform(src, dst)
        matrix_inv = cv2.getPerspectiveTransform(dst, src)
        blend_line = cv2.warpPerspective(blend_color, matrix, [img_x, img_y])
        return blend_line

    def img_binary(self, blend_line):
        bin = cv2.cvtColor(blend_line, cv2.COLOR_BGR2GRAY)
        binary_line = np.zeros_like(bin)
        binary_line[bin != 0] = 1
        return binary_line

    def img_CB(self, data):
        img = self.bridge.compressed_imgmsg_to_cv2(data)

        blend_color = self.detect_color(img)

        blend_line = self.img_warp(img, blend_color)
        binary_line = self.img_binary(blend_line)

        binary_line_img_msg = self.bridge.cv2_to_compressed_imgmsg(binary_line)
        self.pub.publish(binary_line_img_msg)
        cv2.imshow("img", img)
        cv2.imshow("binary_line", binary_line)
        cv2.waitKey(1)


if __name__ == "__main__":
    binary_lines = Binary_Line()
    rospy.spin()
