#!/usr/bin/env python3

import rospy
from morai_msgs.msg import ObjectStatusList
import os


class Object:
    def __init__(self):
        rospy.init_node("object_node")
        self.obj_sub = rospy.Subscriber(
            "/Object_topic", ObjectStatusList, self.object_CB
        )

    # object 정보 콜백 함수
    def object_CB(self, msg):
        self.obj_msg = ObjectStatusList()
        self.obj_msg = msg
        os.system("clear")
        for obj in self.obj_msg.obstacle_list:
            print(f"Object ID: {obj.unique_id}")
            print(f"Object Type: {obj.type}")
            print(f"Object Pos: ")
            print(f"    x : {obj.position.x}")
            print(f"    y : {obj.position.y}")
            print(f"    z : {obj.position.z}")


if __name__ == "__main__":
    object = Object()
    rospy.spin()
