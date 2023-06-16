#!/usr/bin/env python3

import numpy as np
import cv2
import socket
import time
from lib.cam_util import UDP_CAM_Parser
import os,json

class Bird_Eye_View:
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
            cv2.imshow("cam", img_cam)
            cv2.imshow("warp_img",warp_img)
            cv2.waitKey(1)

if __name__ == "__main__":
    bird_eye_view = Bird_Eye_View()
    while True :
        bird_eye_view.main()
