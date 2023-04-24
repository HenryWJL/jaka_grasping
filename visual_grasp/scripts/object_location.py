#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import yaml
import transforms3d as tfs
from geometry_msgs.msg import TwistStamped

object2cam = None


def callback():
    pass
    # paste the codes in 'object_recognition' here


def detection_listener():
    # paste the codes in 'object_recognition' here
    global object2cam
    T_object2cam = None
    R_object2cam = None
    object2cam = np.column_stack((R_object2cam, T_object2cam))
    object2cam = np.row_stack((object2cam, np.array([0, 0, 0, 1])))


def location_publisher():
    with open('~/handeye_calibration/yaml/camera_to_base_matrix.yaml', 'r') as f:
        cam2base = yaml.load(f.read(), Loader=yaml.FullLoader)
        cam2base = np.array(cam2base)
    pub = rospy.Publisher('object_pose', TwistStamped, queue_size=10)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        object2base = np.matmul(object2cam, cam2base)
        R_object2base = object2base[:3, :3]
        T_object2base = object2base[:3, 3]
        rx, ry, rz = tfs.euler.mat2euler(R_object2base)
        pose = TwistStamped()
        pose.twist.linear.x = T_object2base[0]
        pose.twist.linear.y = T_object2base[1]
        pose.twist.linear.z = T_object2base[2]
        pose.twist.angular.x = rx
        pose.twist.angular.y = ry
        pose.twist.angular.z = rz
        pub.publish(pose)
        rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('object_location', anonymous=True)
        detection_listener()
        location_publisher()
    except (rospy.ROSInitException, rospy.ROSInterruptException):
        print("Failed to detect or locate objects!")




