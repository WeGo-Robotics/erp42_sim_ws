#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import numpy as np
import cv2
import os
import matplotlib.pyplot as plt


class Meter_Per_Pixel:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node("sliding_window_node")
        self.pub = rospy.Publisher(
            "/sliding_windows/compressed", CompressedImage, queue_size=10
        )
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.img_CB)
        self.left_fit = [0, 0, 0, 0, 0]
        self.right_fit = [0, 0, 0, 0, 0]

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
        return blend_line, img_x, img_y

    def img_binary(self, blend_line):
        bin = cv2.cvtColor(blend_line, cv2.COLOR_BGR2GRAY)
        binary_line = np.zeros_like(bin)
        binary_line[bin != 0] = 1
        return binary_line

    def window_search(self, binary_line):
        # histogram을 생성합니다.
        # y축 기준 절반 아래 부분만을 사용하여 x축 기준 픽셀의 분포를 구합니다.
        bottom_half_y = binary_line.shape[0] / 2
        histogram = np.sum(binary_line[int(bottom_half_y) :, :], axis=0)
        # 히스토그램을 절반으로 나누어 좌우 히스토그램의 최대값의 인덱스를 반환합니다.
        midpoint = np.int(histogram.shape[0] / 2)
        left_x_base = np.argmax(histogram[:midpoint])
        right_x_base = np.argmax(histogram[midpoint:]) + midpoint
        # show histogram
        # plt.hist(histogram)
        # plt.show()

        left_x_current = left_x_base
        right_x_current = right_x_base

        out_img = np.dstack((binary_line, binary_line, binary_line)) * 255

        ## window parameter
        # 적절한 윈도우의 개수를 지정합니다.
        # 개수가 너무 적으면 정확하게 차선을 찾기 힘듭니다.
        # 개수가 너무 많으면 연산량이 증가하여 시간이 오래 걸립니다.
        nwindows = 10
        window_height = np.int(binary_line.shape[0] / nwindows)
        # 윈도우의 너비를 지정합니다. 윈도우가 옆 차선까지 넘어가지 않게 사이즈를 적절히 지정합니다.
        margin = 100
        # 탐색할 최소 픽셀의 개수를 지정합니다.
        min_pix = 30

        lane_pixel = binary_line.nonzero()
        lane_pixel_y = np.array(lane_pixel[0])
        lane_pixel_x = np.array(lane_pixel[1])

        # pixel index를 담을 list를 만들어 줍니다.
        left_lane_idx = []
        right_lane_idx = []

        # Step through the windows one by one
        for window in range(nwindows):
            # window boundary를 지정합니다. (가로)
            win_y_low = binary_line.shape[0] - (window + 1) * window_height
            win_y_high = binary_line.shape[0] - window * window_height
            # print("check param : \n",window,win_y_low,win_y_high)

            # position 기준 window size
            win_x_left_low = left_x_current - margin
            win_x_left_high = left_x_current + margin
            win_x_right_low = right_x_current - margin
            win_x_right_high = right_x_current + margin

            # window 시각화입니다.
            cv2.rectangle(
                out_img,
                (win_x_left_low, win_y_low),
                (win_x_left_high, win_y_high),
                (0, 255, 0),
                2,
            )
            cv2.rectangle(
                out_img,
                (win_x_right_low, win_y_low),
                (win_x_right_high, win_y_high),
                (0, 0, 255),
                2,
            )

            # 왼쪽 오른쪽 각 차선 픽셀이 window안에 있는 경우 index를 저장합니다.
            good_left_idx = (
                (lane_pixel_y >= win_y_low)
                & (lane_pixel_y < win_y_high)
                & (lane_pixel_x >= win_x_left_low)
                & (lane_pixel_x < win_x_left_high)
            ).nonzero()[0]
            good_right_idx = (
                (lane_pixel_y >= win_y_low)
                & (lane_pixel_y < win_y_high)
                & (lane_pixel_x >= win_x_right_low)
                & (lane_pixel_x < win_x_right_high)
            ).nonzero()[0]

            # Append these indices to the lists
            left_lane_idx.append(good_left_idx)
            right_lane_idx.append(good_right_idx)

            # window내 설정한 pixel개수 이상이 탐지되면, 픽셀들의 x 좌표 평균으로 업데이트 합니다.
            if len(good_left_idx) > min_pix:
                left_x_current = np.int(np.mean(lane_pixel_x[good_left_idx]))
            if len(good_right_idx) > min_pix:
                right_x_current = np.int(np.mean(lane_pixel_x[good_right_idx]))

        # np.concatenate(array) => axis 0으로 차원 감소 시킵니다.(window개수로 감소)
        left_lane_idx = np.concatenate(left_lane_idx)
        right_lane_idx = np.concatenate(right_lane_idx)

        # window 별 좌우 도로 픽셀 좌표입니다.
        left_x = lane_pixel_x[left_lane_idx]
        left_y = lane_pixel_y[left_lane_idx]
        right_x = lane_pixel_x[right_lane_idx]
        right_y = lane_pixel_y[right_lane_idx]

        # 좌우 차선 별 2차함수 계수를 추정합니다.
        if len(left_x) == 0 or len(left_x) == 0:
            left_fit = self.left_fit
            right_fit = self.right_fit

        else:
            left_fit = np.polyfit(left_y, left_x, 2)
            right_fit = np.polyfit(right_y, right_x, 2)
            self.left_fit = left_fit
            self.right_fit = right_fit

        # 좌우 차선 별 추정할 y좌표입니다.
        plot_y = np.linspace(0, binary_line.shape[0] - 1, 5)
        # 좌우 차선 별 2차 곡선을 추정합니다.
        left_fit_x = left_fit[0] * plot_y**2 + left_fit[1] * plot_y + left_fit[2]
        right_fit_x = right_fit[0] * plot_y**2 + right_fit[1] * plot_y + right_fit[2]
        center_fit_x = (right_fit_x + left_fit_x) / 2

        # # window안의 lane을 black 처리합니다.
        # out_img[lane_pixel_y[left_lane_idx], lane_pixel_x[left_lane_idx]] = (0, 0, 0)
        # out_img[lane_pixel_y[right_lane_idx], lane_pixel_x[right_lane_idx]] = (0, 0, 0)

        # 양쪽 차선 및 중심 선 pixel 좌표(x,y)로 변환합니다.
        center = np.asarray(tuple(zip(center_fit_x, plot_y)), np.int32)
        right = np.asarray(tuple(zip(right_fit_x, plot_y)), np.int32)
        left = np.asarray(tuple(zip(left_fit_x, plot_y)), np.int32)

        cv2.polylines(out_img, [right], False, (0, 255, 0), thickness=5)
        cv2.polylines(out_img, [left], False, (0, 0, 255), thickness=5)
        sliding_window_img = out_img
        return sliding_window_img, left, right, center, left_x, left_y, right_x, right_y

    def meter_per_pixel(
        self,
    ):
        world_warp = np.array(
            [[97, 1610], [109, 1610], [109, 1606], [97, 1606]], np.float32
        )
        pix_x = 640
        pix_y = 480
        meter_x = np.sum((world_warp[0] - world_warp[3]) ** 2)
        meter_y = np.sum((world_warp[0] - world_warp[1]) ** 2)
        meter_per_pix_x = meter_x / pix_x
        meter_per_pix_y = meter_y / pix_y
        return meter_per_pix_x, meter_per_pix_y

    def img_CB(self, data):
        img = self.bridge.compressed_imgmsg_to_cv2(data)

        blend_color, img_x, img_y = self.detect_color(img)

        blend_line = self.img_warp(img, blend_color)

        binary_line = self.img_binary(blend_line)

        (
            sliding_window_img,
            left,
            right,
            center,
            left_x,
            left_y,
            right_x,
            right_y,
        ) = self.window_search(binary_line)
        meter_per_pix_x, meter_per_pix_y = self.meter_per_pixel(img_x, img_y)

        os.system("clear")
        print(f"------------------------------")
        print(f"left : {left}")
        print(f"right : {right}")
        print(f"center : {center}")
        print(f"left_x : {left_x}")
        print(f"left_y : {left_y}")
        print(f"right_x : {right_x}")
        print(f"right_y : {right_y}")
        print(f"meter_per_pix_x : {meter_per_pix_x}")
        print(f"meter_per_pix_y : {meter_per_pix_y}")
        print(f"------------------------------")
        sliding_window_msg = self.bridge.cv2_to_compressed_imgmsg(sliding_window_img)
        self.pub.publish(sliding_window_msg)
        cv2.imshow("img", img)
        cv2.imshow("sliding_window_img", sliding_window_img)
        cv2.waitKey(1)


if __name__ == "__main__":
    meter_per_pixel = Meter_Per_Pixel()
    rospy.spin()
