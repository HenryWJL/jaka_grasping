#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This is used for objects localization. The published pose
is the position of the object w.r.t the robot's base_link.
"""

import rospy
import numpy as np
import yaml
import transforms3d as tfs
import time
from geometry_msgs.msg import TwistStamped
from tf2_msgs.msg import TFMessage


def callback(poses):
    # Obtaining object-to-camera matrix
    global object2cam
    if poses is None:
        rospy.logwarn("No object data!")

    else:
        poses = poses.transforms[0].transform
        T_object2cam = np.array([[poses.translation.x], [poses.translation.y], [poses.translation.z]])
        R_object2cam = tfs.quaternions.quat2mat((poses.rotation.w, poses.rotation.x,
                                                 poses.rotation.y, poses.rotation.z))
        object2cam = np.column_stack((R_object2cam, T_object2cam))
        object2cam = np.row_stack((object2cam, np.array([0, 0, 0, 1])))


if __name__ == '__main__':
    rospy.init_node('object_location_aruco', anonymous=True)
    rospy.Subscriber('/tf', TFMessage, callback, queue_size=10)
    pub = rospy.Publisher('/object_pose', TwistStamped, queue_size=10)
    rate = rospy.Rate(5)
    # Variables
    object2cam = None  # The object-to-camera matrix
    with open('../../hand_eye_calibration/yaml/camera_to_base_matrix.yaml', 'r') as f:
        cam2base = yaml.load(f.read(), Loader=yaml.FullLoader)
        cam2base = np.array(cam2base)  # The camera-to-base matrix

    rospy.sleep(1)

    while not rospy.is_shutdown():
        try:
            if cam2base is None:
                rospy.logerr("No object_to_camera matrix data!")
                break

            if object2cam is None:
                rospy.logwarn("No object data available!")
                time.sleep(1)
                continue
            # Calculating object-to-base matrix
            object2base = np.matmul(cam2base, object2cam)
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
            rospy.loginfo(f'Object pose: {[T_object2base[0], T_object2base[1], T_object2base[2], rx, ry, rz]}')

            rate.sleep()
            time.sleep(1)

        except rospy.ROSInterruptException:
            rospy.logwarn("No object data available!")
            time.sleep(2)
            continue
