#!/usr/bin/env python3
  
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import *
from time import *

class lidarPaser:
    def __init__(self):
        rospy.init_node('e_stop', anonymous=True)
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel',Twist, queue_size=1)
        self.roi_deg = 30
        self.roi_range = 0.5
        self.ctrl_flag  = 0
        self.start_time = time()
        rospy.spin()

    def callback(self, data):
        self.end_time = time()
        # sense part
        cmd_msg = Twist()
        lidar_msg = LaserScan()
        print(f'data.angle_min: ',data.angle_min)
        print(f'data.angle_max: ',data.angle_max)
        print(f'data.angle_increment: ',data.angle_increment)
        print(f'data.range_max: ',data.range_max)
        print(f'data.range_min: ',data.range_min)
        print(f'data.ranges: ',data.ranges)

        angle_rad = [data.angle_min + data.angle_increment * index for index, value in enumerate(data.ranges)]
        angle_deg = [angle * 180 / pi for angle in angle_rad]
        
        # think part
        for index, angle in enumerate(angle_deg):
            if -self.roi_deg <= angle <= self.roi_deg and data.ranges(index) < self.roi_range :
                self.ctrl_flag  += 1
                
        # ctrl part
        if self.ctrl_flag  > 30 :
            cmd_msg.linear.x = 0
        else :
            cmd_msg.linear.x = 0.3
            
        cmd_msg.angular.z = 0
        
        if self.end_time - self.start_time >= 1000:
            self.start_time = self.end_time
            self.cmd_pub.publish(cmd_msg)
    

if __name__ == '__main__':
    try:
        lidar = lidarPaser()
    except rospy.ROSInterruptException:
        pass