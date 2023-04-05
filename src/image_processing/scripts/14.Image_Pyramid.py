#!/usr/bin/env python3

# import libraries
import cv2  # OpenCV library for image processing
import rospy  # ROS Python library for working with ROS
from sensor_msgs.msg import Image  # ROS Image message type
from cv_bridge import CvBridge  # Library to convert between ROS and OpenCV images
import rospkg  # ROS package library


class Image_Pyramid:
    def __init__(self):
        # initialize a ROS node
        rospy.init_node("image_node")

        # Define titles for each edge detection method
        self.titles = ["image", "lower_resolution", "higher_resolution"]

        # set the publishing rate to 10 Hz
        self.rate = rospy.Rate(10)

        # Get the file path of the "image_processing" package
        rospack = rospkg.RosPack()
        self.file_path = rospack.get_path("image_processing")
        self.file_path += "/scripts/"

        # Set the file path for the image file to be used
        self.image_path = self.file_path + "Lenna.png"

        # Initialize bridge to convert between ROS and OpenCV images
        self.bridge = CvBridge()

        # create a list of publishers for each of the four transformed images
        self.pubs = []
        for title in self.titles:
            pub = rospy.Publisher("/image_topic/" + title, Image, queue_size=1)
            self.pubs.append(pub)

    def main(self):
        # Load an image from a file using OpenCV
        image = cv2.imread(self.image_path)

        # Apply pyrDown and pyrUp to create two new images with lower and higher resolution respectively
        lower_reso = cv2.pyrDown(image)
        higher_reso = cv2.pyrUp(image)

        images = [image, lower_reso, higher_reso]

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
    image_pyramid = Image_Pyramid()
    try:
        # run the main function of the Image_Transformation_Traslation class until the ROS node is shut down
        while not rospy.is_shutdown():
            image_pyramid.main()

    except rospy.ROSInterruptException:
        pass
