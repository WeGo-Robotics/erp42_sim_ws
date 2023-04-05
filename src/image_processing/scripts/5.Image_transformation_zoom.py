#!/usr/bin/env python3

# import libraries
import cv2  # OpenCV library for image processing
import rospy  # ROS Python library for working with ROS
from sensor_msgs.msg import Image  # ROS Image message type
from cv_bridge import CvBridge  # Library to convert between ROS and OpenCV images
import rospkg  # ROS package library


# create a class for image transformation and zoom
class Image_Transformation_Zoom:
    def __init__(self):
        # initialize a ROS node
        rospy.init_node("image_node")

        # define a list of titles for the images to be displayed
        self.titles = ["original", "shrink", "zoom1", "zoom2"]

        # set the publishing rate to 10 Hz
        self.rate = rospy.Rate(10)

        # Get the file path of the "image_processing" package
        rospack = rospkg.RosPack()
        self.file_path = rospack.get_path("image_processing")
        self.file_path += "/scripts/"

        # Set the file path for the image file to be used
        self.image_path = self.file_path + "opencv-logo.png"

        # Initialize bridge to convert between ROS and OpenCV images
        self.bridge = CvBridge()

        # create a list of publishers for each of the four transformed images
        self.pubs = []
        for title in self.titles:
            pub = rospy.Publisher("/image_topic/" + title, Image, queue_size=1)
            self.pubs.append(pub)

    # create the main function for the image transformation and zoom
    def main(self):
        # read in the image file
        image = cv2.imread(self.image_path)

        # Publish shrink image
        shrink = cv2.resize(image, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)

        # Publish zoom1 image
        height, width = image.shape[:2]
        zoom1 = cv2.resize(
            image, (width * 2, height * 2), interpolation=cv2.INTER_CUBIC
        )

        # Publish zoom2 image
        zoom2 = cv2.resize(image, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)

        # create a list of the original image and the transformed images
        images = [image, shrink, zoom1, zoom2]

        # publish each image to its corresponding topic
        for i, image in enumerate(images):
            # convert the OpenCV image to a ROS image message
            ros_img = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")

            # publish the ROS image message to the corresponding topic
            self.pubs[i].publish(ros_img)

            # wait for the next publishing cycle
            self.rate.sleep()


# create an instance of the Image_transformation_zoom class
if __name__ == "__main__":
    image_transformation_zoom = Image_Transformation_Zoom()
    try:
        # run the main function of the ImageThreshold class until the ROS node is shut down
        while not rospy.is_shutdown():
            image_transformation_zoom.main()

    except rospy.ROSInterruptException:
        pass
