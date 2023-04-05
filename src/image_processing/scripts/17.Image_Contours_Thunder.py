#!/usr/bin/env python3

# import libraries
import cv2  # OpenCV library for image processing
import rospy  # ROS Python library for working with ROS
from sensor_msgs.msg import Image  # ROS Image message type
from cv_bridge import CvBridge  # Library to convert between ROS and OpenCV images
import rospkg  # ROS package library
import numpy as np


class Image_Contours_Thunder:
    def __init__(self):
        # initialize a ROS node
        rospy.init_node("image_node")

        # define a list of titles for the images to be displayed
        self.titles = ["Original", "Result"]

        # set the publishing rate to 10 Hz
        self.rate = rospy.Rate(10)

        # Get the file path of the "image_processing" package
        rospack = rospkg.RosPack()
        self.file_path = rospack.get_path("image_processing")
        self.file_path += "/scripts/"

        # Set the file path for the image file to be used
        self.image_path = self.file_path + "thunder.png"
        # Initialize bridge to convert between ROS and OpenCV images
        self.bridge = CvBridge()
        # create a list of publishers for each of the four transformed images
        self.pubs = []

        for title in self.titles:
            # create a publisher for each title in the titles list
            pub = rospy.Publisher("/image_topic/" + title, Image, queue_size=1)
            # append the publisher to the pubs list
            self.pubs.append(pub)

    def main(self):
        # Load the image
        image = cv2.imread(self.image_path)
        image1 = image.copy()

        # Convert the image to grayscale and threshold it
        image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(image_gray, 220, 255, 0)

        # Find contours in the thresholded image
        contours, hierachy = cv2.findContours(
            thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )
        cnt = contours[0]

        # Draw shapes around the contour
        # Straight Rectangle
        x, y, w, h = cv2.boundingRect(cnt)
        image1 = cv2.rectangle(image1, (x, y), (x + w, y + h), (0, 255, 0), 3)  # green

        # Rotated Rectangle
        rect = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rect)
        box = np.intp(box)
        image1 = cv2.drawContours(image1, [box], 0, (0, 0, 255), 3)  # blue

        # Minimum Enclosing Circle
        (x, y), radius = cv2.minEnclosingCircle(cnt)
        center = (int(x), int(y))
        radius = int(radius)
        image1 = cv2.circle(image1, center, radius, (255, 255, 0), 3)  # yellow

        # Fitting an Ellipse
        ellipse = cv2.fitEllipse(cnt)
        image1 = cv2.ellipse(image1, ellipse, (255, 0, 0), 3)  # red

        # Display the images
        images = [image, image1]
        # publish each image to its corresponding topic
        for i, image in enumerate(images):
            # convert the OpenCV image to a ROS image message
            ros_img = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")

            # publish the ROS image message to the corresponding topic
            self.pubs[i].publish(ros_img)

            # wait for the next publishing cycle
            self.rate.sleep()


# create an instance of the Image_Transformation_Traslation class
if __name__ == "__main__":
    image_contours_thunder = Image_Contours_Thunder()
    try:
        # run the main function of the Image_Transformation_Traslation class until the ROS node is shut down
        while not rospy.is_shutdown():
            image_contours_thunder.main()

    except rospy.ROSInterruptException:
        pass
