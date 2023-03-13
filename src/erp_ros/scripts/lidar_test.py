#!/usr/bin/env python3

import rospy
import socket
import numpy as np
import time
import matplotlib.pyplot as plt
from lib.lidar_util import UDP_LIDAR_Parser
import os,json
from sensor_msgs.msg import PointCloud, PointField, ChannelFloat32
from std_msgs.msg import Header
# import open3d as o3d
import struct

path = os.path.dirname( os.path.abspath( __file__ ) )

with open(os.path.join(path,("params.json")),'r') as fp :
    params = json.load(fp)

params=params["params"]
user_ip = params["user_ip"]
lidar_port = params["lidar_dst_port"]


params_lidar = {
    "Range" : 90, #min & max range of lidar azimuths
    "CHANNEL" : 16, #verticla channel of a lidar
    "localIP": user_ip,
    "localPort": lidar_port,
    "Block_SIZE": int(1206)
}
a = PointCloud()
class Lidar():
    def __init__(self):
        self.udp_lidar = UDP_LIDAR_Parser(ip=params_lidar["localIP"], port=params_lidar["localPort"], params_lidar=params_lidar)
        rospy.init_node('lidar_node')
        self.pub = rospy.Publisher('lidar3D',PointCloud,queue_size=10)
        self.lidar_msg = PointCloud()
        self.header = Header()
        self.lidar_msg.header.frame_id = 'velodyne'
        self.main()

    def main(self):    

        while not rospy.is_shutdown() :
            points = []
            if self.udp_lidar.is_lidar ==True:            
                self.x=self.udp_lidar.x
                self.y=self.udp_lidar.y
                self.z=self.udp_lidar.z
                self.intensity=self.udp_lidar.Intensity 
                self.distance=self.udp_lidar.Distance
                self.xyz1 = np.concatenate([
                    self.x.reshape([-1, 1]),
                    self.y.reshape([-1, 1]),
                    self.z.reshape([-1, 1])
                ], axis=1).T.astype(np.float32)

                # print(self.xyz1.T)
                cloud_msg = PointCloud()
        cloud_msg.header = self.header
        cloud_msg.points = []
        for p in points:
            point = [p[0], p[1], p[2]]
            cloud_msg.points.append(point)
        cloud_msg.channels = []
        intensity_channel = ChannelFloat32()
        intensity_channel.name = "intensity"
        intensity_channel.values = [p[3] for p in points]
        cloud_msg.channels.append(intensity_channel)
        ring_channel = ChannelFloat32()
        ring_channel.name = "ring"
        ring_channel.values = [p[4] for p in points]
        cloud_msg.channels.append(ring_channel)

        # PointCloud 메시지 발행
        self.pub.publish(cloud_msg)
                
    
if __name__ == '__main__':
    lidar_class = Lidar()
        
