#!/usr/bin/env python3

import numpy as np
import cv2
import socket
import time
from lib.cam_util import UDP_CAM_Parser
import os,json

class Binary_Line:
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

    def img_binary(self, blend_color):
        bin = cv2.cvtColor(blend_color, cv2.COLOR_BGR2GRAY)
        binary_line = np.zeros_like(bin)
        binary_line[bin != 0] = 1
        return binary_line

    def main(self):    
        if self.udp_cam.is_img==True :
            img_cam = self.udp_cam.raw_img
            warp_img = self.img_warp(img_cam)
            blend_color = self.detect_color(warp_img)
            binary_line = self.img_binary(blend_color)

            cv2.imshow("cam", img_cam)
            cv2.imshow("warp_img",warp_img)
            cv2.imshow("blend_color",blend_color)
            cv2.imshow("binary_line",binary_line)
            cv2.waitKey(1)


if __name__ == "__main__":
    binary_lines = Binary_Line()
    while True :
        binary_lines.main()
