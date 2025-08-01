import rospy
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
import numpy as np
from morai_msgs.msg import CtrlCmd,EgoVehicleStatus
from math import *

class Sub_class:
    def __init__(self):
        rospy.init_node("wego_sub_node")
        rospy.Subscriber("/image_jpeg/compressed",CompressedImage,self.img_cb)
        rospy.Subscriber("/Ego_topic",EgoVehicleStatus,self.ego_cb)
        self.pub_node = rospy.Publisher("/ctrl_cmd",CtrlCmd,queue_size=1)
        self.current_vel_x = 0
        self.bridge = CvBridge()
        self.cmd_msg =CtrlCmd()

    def ego_cb(self,msg:EgoVehicleStatus):
        self.current_vel_x = msg.velocity.x 
    def img_cb(self,msg:CompressedImage):
        img = self.bridge.compressed_imgmsg_to_cv2(msg) 
        y,x,channel = img.shape
        hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        y_lower = np.array([15,100,100])
        y_upper = np.array([45,255,255])
        b_lower = np.array([100,100,100])
        b_upper = np.array([140,255,255])
        y_masked_img = cv2.inRange(hsv,y_lower,y_upper)
        b_masked_img = cv2.inRange(hsv,b_lower,b_upper)
        combined_img = cv2.bitwise_or(y_masked_img,b_masked_img)
        bit_img = cv2.bitwise_and(img,img,mask=combined_img)
        # self.view("img",img)
        # self.view("bit_img",bit_img)

        x1_margin=80
        x2_margin=255
        y2_margin=300
        src1=[x1_margin,y]
        src2=[x2_margin,y2_margin]
        src3=[x-x2_margin,y2_margin]
        src4=[x-x1_margin,y]
        srcs = np.float32([src1,src2,src3,src4])

        dst1=[x1_margin,y]
        dst2=[x1_margin,0]
        dst3=[x-x1_margin,0]
        dst4=[x-x1_margin,y]
        dsts=np.float32([dst1,dst2,dst3,dst4])

        matrix = cv2.getPerspectiveTransform(srcs,dsts)
        warp_img = cv2.warpPerspective(bit_img,matrix,((x,y)))
        gray_warp_img = cv2.cvtColor(warp_img,cv2.COLOR_BGR2GRAY)
        bin_img = np.zeros_like(gray_warp_img)
        bin_img[gray_warp_img!=0]=1
        histogram = np.sum(bin_img,axis=0)
        histogram[histogram<100]=0
        left_hist =histogram[0:x//2]
        right_hist = histogram[x//2:x]
        nonzero_left = np.nonzero(left_hist)[0]
        nonzero_right = np.nonzero(right_hist)[0]
        try:
            left_avg = (nonzero_left[0]+nonzero_left[-1]) //2
            right_avg = ( ( (nonzero_right[0]+nonzero_right[-1]) //2) +x//2)
            center_avg = (left_avg+right_avg) //2
        except:
            center_avg = x//2

        cv2.line(warp_img,[x//2,0],[x//2,y],(0,255,0),7)
        cv2.line(warp_img,[center_avg,0],[center_avg,y],(0,0,255),3)

        diff_center = x//2 - center_avg
        steer = diff_center*pi/x
        self.view("warp_img",warp_img)
        if self.current_vel_x>3:
            self.cmd_msg.accel = 0.0
            self.cmd_msg.brake=0.1
        else:
            self.cmd_msg.accel = 0.2
            self.cmd_msg.brake=0.0
            self.cmd_msg.steering = steer * 0.5
        # print(steer)
        self.pub_node.publish(self.cmd_msg)

    def view(self,window_name,img):
        cv2.namedWindow(window_name,cv2.WINDOW_NORMAL)
        cv2.imshow(window_name,img)
        cv2.waitKey(1)

sub_class = Sub_class()
rospy.spin()