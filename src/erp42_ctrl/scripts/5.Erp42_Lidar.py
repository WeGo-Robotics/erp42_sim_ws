#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


def lidar3d_callback(data):
    for p in pc2.read_points(data, skip_nans=True):
        x = p[0]
        y = p[1]
        z = p[2]
        rospy.loginfo("Lidar 3D data: x={}, y={}, z={}".format(x, y, z))


# When the main script is executed, run the following code.
if __name__ == "__main__":
    rospy.init_node("lidar3d_subscriber")

    # Subscribe to Morai PointCloud2 topic
    rospy.Subscriber("/lidar3D", PointCloud2, lidar3d_callback)

    rospy.spin()
