#!/usr/bin/env python3

# import libraries
import cv2 # OpenCV library for image processing
import rospy # ROS Python library for working with ROS
from sensor_msgs.msg import Image # ROS Image message type
from cv_bridge import CvBridge # Library to convert between ROS and OpenCV images
import rospkg # ROS package library
import numpy as np

class Image_Template_Matching:
    def __init__(self):
        # initialize a ROS node
        rospy.init_node('image_publisher')

        # define a list of titles for the images to be displayed
        self.titles = ['res','img']

        # set the publishing rate to 10 Hz
        self.rate = rospy.Rate(10)

        # Get the file path of the "Image_Processing" package
        rospack = rospkg.RosPack()
        self.file_path = rospack.get_path("image_processing")
        self.file_path += "/scripts/"

        # Set the file path for the image file to be used
        self.messi_Face_image_path = self.file_path + "messi_Face.png"
        self.messi_image_path = self.file_path + "messi.jpeg"

        # Initialize bridge to convert between ROS and OpenCV images
        self.bridge = CvBridge()

        # create a list of publishers for each of the four transformed images
        self.pubs = []
        for title in self.titles:
            # create a publisher for each title in the titles list
            pub = rospy.Publisher('/image_topic/' + title, Image, queue_size=1)
            # append the publisher to the pubs list
            self.pubs.append(pub)


    def main(self):
        # Load the input image
        image = cv2.imread(self.messi_image_path,0)
        # Create a copy of the original image  
        img2=image.copy()
        # Load the template image from the file path and convert to grayscale
        image_template = cv2.imread(self.messi_Face_image_path,0)  
        # Get the width and height of the template image
        w,h = image_template.shape[::-1]   

        # Template Match Method
        methods = ['cv2.TM_CCOEFF', 'cv2.TM_CCOEFF_NORMED', 'cv2.TM_CCORR', 'cv2.TM_CCORR_NORMED', 'cv2.TM_SQDIFF', 'cv2.TM_SQDIFF_NORMED'] 
        for meth in methods:
            img = img2.copy()
            method = eval(meth)
            # Perform the template matching operation using the specified method
            res = cv2.matchTemplate (img, image_template,method)
            # Find the minimum and maximum values in the result matrix  
            min_val,max_val,min_loc, max_loc= cv2.minMaxLoc(res)  

            if method in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]:
                # If the method is TM_SQDIFF or TM_SQDIFF_NORMED, get the top-left location of the template
                top_left = min_loc   
            else:
                # Otherwise, get the top-left location of the maximum value in the result matrix
                top_left = max_loc   
         # Calculate the bottom-right location of the template
        bottom_right = (top_left[0]+w, top_left[1]+h) 
        # Draw a rectangle around the detected template on the original image
        cv2.rectangle(img, top_left, bottom_right,255,5)  
        # Create a list of the resulting images
        images = [res,img]   

        # publish each image to its corresponding topic
        for i, image in enumerate(images):
            # Convert the OpenCV image to a ROS message
            if len(image.shape) == 2:
                # Grayscale image
                ros_image = self.bridge.cv2_to_imgmsg(image)
            else:
                # Color image
                ros_image = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
            
            # publish the ROS image message to the corresponding topic
            self.pubs[i].publish(ros_image)
            
            # wait for the next publishing cycle
            self.rate.sleep()



# create an instance of the Image_Histogram_CLAHE class
if __name__ == '__main__':
    image_Template_Matching = Image_Template_Matching()

    try:
        # run the main function of the Image_Histogram_CLAHE class until the ROS node is shut down
        while not rospy.is_shutdown():
            image_Template_Matching.main()

    except rospy.ROSInterruptException:
        pass
