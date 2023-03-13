#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class Draw:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("image_node", anonymous=False)
        # Create a publisher to publish the image
        self.image_pub_color = rospy.Publisher("draw_img", Image, queue_size=10)
        self.rate = rospy.Rate(10)

    def main(self):
        # Initialize bridge to convert between ROS and OpenCV images
        self.bridge = CvBridge()

        self.height = 480
        self.width = 640
        self.white = [255, 255, 255]
        self.blue = [255, 0, 0]
        self.green = [0, 255, 0]
        self.red = [0, 0, 255]
        self.cyan = [255, 255, 0]
        self.magenta = [255, 0, 255]
        self.yellow = [0, 255, 255]
        self.color = np.zeros((self.height, self.width, 3), np.uint8)

        self.pts1 = np.array([[300, 150], [400, 150], [350, 250]])
        self.pts2 = np.array([[425, 200], [500, 150], [500, 250]])

        self.color[175, 125] = self.white
        self.color[0:50, 50:100] = self.blue
        self.color[50:100, 100:150] = self.green
        self.color[100:150, 150:200] = self.red
        self.color[200:400, 100:110] = self.cyan

        cv2.line(self.color, (100, 200), (300, 400), self.magenta, 5)
        cv2.line(self.color, (150, 300), (150, 300), self.yellow, 10)
        cv2.circle(self.color, (300, 400), 50, self.blue, 5)
        cv2.rectangle(self.color, (400, 300), (600, 400), self.green, 5)
        cv2.polylines(self.color, [self.pts1], True, self.red, 2)
        cv2.fillPoly(self.color, [self.pts2], self.yellow)
        cv2.putText(
            self.color,
            "Hello World",
            (225, 100),
            cv2.FONT_HERSHEY_COMPLEX,
            2,
            self.white,
            3,
        )

        img_msg = self.bridge.cv2_to_imgmsg(self.color, "bgr8")
        self.image_pub_color.publish(img_msg)
        self.rate.sleep()


if __name__ == "__main__":
    draw = Draw()
    try:
        while not rospy.is_shutdown():
            draw.main()
    except rospy.ROSInterruptException:
        pass
