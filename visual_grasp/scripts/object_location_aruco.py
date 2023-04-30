#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import yaml
import tf
import transforms3d as tfs
import time
from geometry_msgs.msg import TwistStamped


def location_publisher():
    rospy.init_node('object_location_aruco', anonymous=True)
    with open('./jaka_ws/src/jaka_grasping/handeye_calibration/yaml/camera_to_base_matrix.yaml', 'r') as f:
        cam2base = yaml.load(f.read(), Loader=yaml.FullLoader)
        cam2base = np.array(cam2base)
    pub = rospy.Publisher('object_pose', TwistStamped, queue_size=10)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        try:
            if cam2base is None:
                rospy.logerr("No object_to_camera matrix data!")
                break
            listener = tf.TransformListener()
            T_object2cam, R_object2cam = listener.lookupTransform('/camera_link', 'aruco_marker_frame', rospy.Time(0))
            T_object2cam = np.array((T_object2cam[0], T_object2cam[1], T_object2cam[2]))
            R_object2cam = tfs.quaternions.quat2mat((R_object2cam[0], R_object2cam[1],
                                                     R_object2cam[2], R_object2cam[3]))
            object2cam = np.column_stack((R_object2cam, T_object2cam))
            object2cam = np.row_stack((object2cam, np.array([0, 0, 0, 1])))
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
            rospy.loginfo(f'Object pose: {[T_object2base[0], T_object2base[1], T_object2base[2], rx, ry, rz]}')
            pub.publish(pose)
            rate.sleep()
            time.sleep(2)
        except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("No object data available!")
            continue


if __name__ == '__main__':
    location_publisher()
