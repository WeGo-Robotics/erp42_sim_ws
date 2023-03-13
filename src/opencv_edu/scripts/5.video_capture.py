#!/usr/bin/env python3

import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Define a class to capture and publish images from webcam
class Webcam:
    # Initialize the node and publisher, and configure the webcam
    def __init__(self):
        # Initialize a node with the name "webcam_node"
        rospy.init_node("webcam_node", anonymous=True)
        # Create a publisher to publish image messages
        self.image_pub = rospy.Publisher("webcam_img", Image, queue_size=1)

        # Create a VideoCapture object to access the default camera (index 0)
        self.capture = cv2.VideoCapture(0)
        # Set the width of the frames to 640 pixels
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        # Set the height of the frames to 480 pixels
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        # Set the frame rate to 30 frames per second
        self.capture.set(cv2.CAP_PROP_FPS, 30)

    # Capture and publish the images
    def main(self):
        # Initialize a CvBridge object to convert OpenCV images to ROS images
        self.bridge = CvBridge()
        # Capture a frame from the camera
        _, self.img = self.capture.read()
        # Convert the OpenCV image to a ROS image message
        webcam_img_msg = self.bridge.cv2_to_imgmsg(self.img, "bgr8")
        # Publish the ROS image message
        self.image_pub.publish(webcam_img_msg)


if __name__ == "__main__":
    webcam = Webcam()
    try:
        while not rospy.is_shutdown():
            webcam.main()
    except rospy.ROSInterruptException:
        pass
