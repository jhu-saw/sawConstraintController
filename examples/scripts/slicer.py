#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Point, Vector3, Transform
from ros_igtl_bridge.msg import igtlpointcloud, igtltransform, igtlpoint

import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.distance import cdist

import datetime

m_to_mm = 1E3

class RegistrationObject():
    def measuredCPCallBack(self, data):
        msg = igtlpoint()
        msg.name = "Measured"
        msg.pointdata = Point(
            data.pose.position.x*m_to_mm, data.pose.position.y*m_to_mm, data.pose.position.z*m_to_mm)

        self.igtl_point_pub.publish(msg)

    def servoCPCallBack(self, data):
        msg = igtlpoint()
        msg.name = "Servo"
        msg.pointdata = Point(
            data.pose.position.x*m_to_mm, data.pose.position.y*m_to_mm, data.pose.position.z*m_to_mm)

        self.igtl_point_pub.publish(msg)

    def transformCallback(self, data):
        print('transform')

        # TODO: check name
        transform = data.transform
        transform.translation.x = transform.translation.x/m_to_mm
        transform.translation.y = transform.translation.y/m_to_mm
        transform.translation.z = transform.translation.z/m_to_mm

        print(transform)
        self.transform_pub.publish(transform)

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