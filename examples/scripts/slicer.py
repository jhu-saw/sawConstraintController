#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Point, Vector3, Transform
from ros_igtl_bridge.msg import igtlpointcloud, igtltransform, igtlpoint

import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.distance import cdist

import datetime

class RegistrationObject():
    def poseCallback(self, data):
        msg = igtlpoint()
        msg.name = "Tip"
        msg.pointdata = Point(
            data.pose.position.x, data.pose.position.y, data.pose.position.z)

        self.igtl_point_pub.publish(msg)

    def transformCallback(self, data):
        print('transform')
        print(data)

        # TODO: check name
        transfrom = data.transform
        self.transform_pub.publish(transfrom)

    def pub_igtl_point(self, p_pivot):
        msg = igtlpointcloud()
        msg.name = 'pivot_point'

        msg.pointdata = []
        for i in range(self.p_pivot.shape[0]):
            p = Point(p_pivot[i, 0], p_pivot[i, 1], p_pivot[i, 2])
            msg.pointdata.append(p)

        self.igtl_pointcloud_pub.publish(msg)

    def registration(self):
        rospy.init_node('ds_registration', anonymous=True)

        # subscribe to robot ee location
        sub = rospy.Subscriber('/simple_robot/measured_cp',
                               PoseStamped, self.poseCallback)
        transform_sub = rospy.Subscriber(
            '/IGTL_TRANSFORM_IN', igtltransform, self.transformCallback)
        # publish to slicer
        self.igtl_point_pub = rospy.Publisher(
            '/IGTL_POINT_OUT', igtlpoint, queue_size=1, latch=True)

        # publish to robot
        self.transform_pub = rospy.Publisher(
            '/simple_robot/transfrom/ee_to_ds', Transform, queue_size=1, latch=True)

        rate = rospy.Rate(20)  # 10Hz

        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == "__main__":
    obj = RegistrationObject()
    obj.registration()