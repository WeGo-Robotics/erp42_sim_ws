#!/usr/bin/env python3

import numpy as np
import cv2
import socket
import time
from lib.cam_util import UDP_CAM_Parser
import os,json


class White_line_Detect:
    def __init__(self):
        path = os.path.dirname( os.path.abspath( __file__ ) )

        with open(os.path.join(path,("params.json")),'r') as fp :
            params = json.load(fp)

        params=params["params"]
        user_ip = params["user_ip"]
        cam_port = params["cam_dst_port"]

        params_cam = {
            "localIP": user_ip,
            "localPort": cam_port,
            "Block_SIZE": int(65000)
        }
        self.udp_cam = UDP_CAM_Parser(ip=params_cam["localIP"], port=params_cam["localPort"], params_cam=params_cam)


    def detect_color(self, img):
        # Convert to HSV color space
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Define range of white color in HSV
        white_lower = np.array([0, 0, 200])
        white_upper = np.array([179, 64, 255])

        # Threshold the HSV image to get only white colors
        white_mask = cv2.inRange(hsv, white_lower, white_upper)
        white_color = cv2.bitwise_and(img, img, mask=white_mask)
        return white_color

    def img_warp(self, img):
        self.img_x, self.img_y = img.shape[1], img.shape[0]

        # ROI
        src_side_offset = [30, 100]
        src_center_offset = [230, 280]
        src = np.float32(
            [
                [src_side_offset[0], self.img_y - src_side_offset[1]],
                [
                    src_center_offset[0],
                    src_center_offset[1],
                ],
                [
                    self.img_x - src_center_offset[0],
                    src_center_offset[1],
                ],
                [
                    self.img_x - src_side_offset[0],
                    self.img_y - src_side_offset[1],
                ],
            ]
        )
        # 아래 2 개 점 기준으로 dst 영역을 설정합니다.
        dst_offset = [80, 0]
        # offset x 값이 작아질 수록 dst box width 증가합니다.
        dst = np.float32(
            [
                [dst_offset[0], self.img_y],
                [dst_offset[0], 0],
                [self.img_x - dst_offset[0], 0],
                [self.img_x - dst_offset[0], self.img_y],
            ]
        )

        # find perspective matrix
        matrix = cv2.getPerspectiveTransform(src, dst)
        matrix_inv = cv2.getPerspectiveTransform(dst, src)
        warp_img = cv2.warpPerspective(img, matrix, [self.img_x, self.img_y])
        return warp_img
        
    def main(self):
        if self.udp_cam.is_img==True :
            img_cam = self.udp_cam.raw_img
            warp_img = self.img_warp(img_cam)
            white_color = self.detect_color(warp_img)
            cv2.imshow("cam", img_cam)
            cv2.imshow("warp_img",warp_img)
            cv2.imshow("white_color",white_color)

            cv2.waitKey(1)


if __name__ == "__main__":
    white_line_detect = White_line_Detect()
    while True :
        white_line_detect.main()

