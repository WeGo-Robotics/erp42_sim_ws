#!/usr/bin/env python3

import rospy
import rospkg
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

# Define the Bitwise_Operation class
class Bitwise_Operation:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node("image_node", anonymous=False)
        # Create a publisher to publish the output image
        self.image_pub = rospy.Publisher("bitwise_img", Image, queue_size=10)
        self.rate = rospy.Rate(10)  # 10Hz

        # Get the file path to the images
        rospack = rospkg.RosPack()
        self.file_path = rospack.get_path("opencv_edu")
        self.file_path += "/scripts/"

        # Load the images
        self.lenna_path = self.file_path + "Lenna.png"
        self.wego_path = self.file_path + "wego.png"

    def main(self):
        # Initialize bridge to convert between ROS and OpenCV images
        bridge = CvBridge()
        # Load the images
        self.lenna = cv2.imread(self.lenna_path, cv2.IMREAD_COLOR)
        self.wego = cv2.imread(self.wego_path, cv2.IMREAD_COLOR)
        # Convert the images to grayscale
        lenna_gray = cv2.cvtColor(self.lenna, cv2.COLOR_BGR2GRAY)
        wego_gray = cv2.cvtColor(self.wego, cv2.COLOR_BGR2GRAY)
        # Apply binary thresholding to the Wego image to create a binary mask
        _, wego_binary = cv2.threshold(wego_gray, 127, 255, cv2.THRESH_BINARY)
        # Invert the binary mask
        _, wego_binary_inv = cv2.threshold(wego_gray, 127, 255, cv2.THRESH_BINARY_INV)
        # Resize the binary mask to match the size of the Lenna image
        resize_img = cv2.resize(wego_binary, (lenna_gray.shape[1], lenna_gray.shape[0]))
        resize_img_inv = cv2.resize(
            wego_binary_inv, (lenna_gray.shape[1], lenna_gray.shape[0])
        )

        # Apply bitwise operations to the Lenna and Wego images
        and_img = cv2.bitwise_and(lenna_gray, resize_img)
        or_img = cv2.bitwise_or(lenna_gray, resize_img)
        xor_img = cv2.bitwise_xor(lenna_gray, resize_img)
        not_img = cv2.bitwise_not(resize_img)

        # Apply bitwise operations to the Lenna and inverted Wego images
        and_img_inv = cv2.bitwise_and(lenna_gray, resize_img_inv)
        or_img_inv = cv2.bitwise_or(lenna_gray, resize_img_inv)
        xor_img_inv = cv2.bitwise_xor(lenna_gray, resize_img_inv)
        not_img_inv = cv2.bitwise_not(resize_img_inv)

        # Concatenate the resulting images horizontally and vertically
        dst1 = cv2.hconcat([and_img, and_img_inv, or_img, or_img_inv])
        dst2 = cv2.hconcat([xor_img, xor_img_inv, not_img, not_img_inv])

        # Convert the image to a ROS image message and publish it
        rst = cv2.vconcat([dst1, dst2])
        img_msg = bridge.cv2_to_imgmsg(rst)
        self.image_pub.publish(img_msg)
        # Sleep to maintain the rate of the node
        self.rate.sleep()
        # Close all open windows created by OpenCV
        cv2.destroyAllWindows()


if __name__ == "__main__":
    bitwise_operation = Bitwise_Operation()
    try:
        while not rospy.is_shutdown():
            bitwise_operation.main()

    except rospy.ROSInterruptException:
        pass
