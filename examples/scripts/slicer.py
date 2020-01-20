#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Point, Vector3, Transform
from ros_igtl_bridge.msg import igtlpointcloud, igtltransform, igtlpoint

import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.distance import cdist

import datetime

class RegistrationObject():
    def measuredCPCallBack(self, data):
        msg = igtlpoint()
        msg.name = "Measured"
        msg.pointdata = Point(
            data.pose.position.x, data.pose.position.y, data.pose.position.z)

        self.igtl_point_pub.publish(msg)

    def servoCPCallBack(self, data):
        msg = igtlpoint()
        msg.name = "Servo"
        msg.pointdata = Point(
            data.pose.position.x, data.pose.position.y, data.pose.position.z)

        self.igtl_point_pub.publish(msg)

    def transformCallback(self, data):
        print('transform')
        print(data)

        # TODO: check name
        transfrom = data.transform
        self.transform_pub.publish(transfrom)

    def registration(self):
        rospy.init_node('ds_registration', anonymous=True)

        # subscribe to robot ee location
        sub_measured = rospy.Subscriber('/simple_robot/measured_cp',
                               PoseStamped, self.measuredCPCallBack)
        sub_servo = rospy.Subscriber('/simple_robot/servo_cp',
                               PoseStamped, self.servoCPCallBack)
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