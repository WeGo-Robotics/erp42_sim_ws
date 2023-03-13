#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import numpy as np
import cv2


class Bird_Eye_View:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node("bird_eye_node")
        self.pub = rospy.Publisher(
            "/bird_eye/compressed", CompressedImage, queue_size=10
        )
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.img_CB)

    def img_warp(self, img):
        img_x, img_y = img.shape[1], img.shape[0]

        # ROI
        src_side_offset = [30, 100]
        src_center_offset = [100, 40]
        src = np.float32(
            [
                [src_side_offset[0], img_y - src_side_offset[1]],
                [
                    img_x / 2 - src_center_offset[0],
                    img_y / 2 + src_center_offset[1],
                ],
                [
                    img_x / 2 + src_center_offset[0],
                    img_y / 2 + src_center_offset[1],
                ],
                [
                    img_x - src_side_offset[0],
                    img_y - src_side_offset[1],
                ],
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
        warp_img = cv2.warpPerspective(img, matrix, [img_x, img_y])
        return warp_img

    def img_CB(self, data):
        img = self.bridge.compressed_imgmsg_to_cv2(data)

        warp_img = self.img_warp(img)

        warp_img_msg = self.bridge.cv2_to_compressed_imgmsg(warp_img)
        self.pub.publish(warp_img_msg)
        cv2.imshow("img", img)
        cv2.imshow("img_warp", warp_img)
        cv2.waitKey(1)


if __name__ == "__main__":
    bird_eye_view = Bird_Eye_View()
    rospy.spin()
