#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import yaml
import transforms3d as tfs
from geometry_msgs.msg import TwistStamped

object_frame = None
object2cam = None


def callback(object_data):
    global object_frame
    object_id = object_data.data[0]
    object_frame = '/object_' + str(object_id)  # get object_frame through topic '/objects'


def detection_listener():
    global object2cam
    rospy.Subscriber("/objects", Float32MultiArray, callback)
    tf_listener = tf.TransformListener()
    (T_object2cam, R_object2cam) = tf_listener.lookupTransform('/camera_color_optical_frame',
                                                               object_frame, rospy.Time(0))
    object2cam = np.column_stack((R_object2cam, T_object2cam))
    object2cam = np.row_stack((object2cam, np.array([0, 0, 0, 1])))


def location_publisher():
    rospy.init_node('object_location', anonymous=True)
    with open('~/handeye_calibration/yaml/camera_to_base_matrix.yaml', 'r') as f:
        cam2base = yaml.load(f.read(), Loader=yaml.FullLoader)
        cam2base = np.array(cam2base)
    pub = rospy.Publisher('object_pose', TwistStamped, queue_size=10)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        try:
            detection_listener()
            if cam2base is None:
                print("No object_to_camera matrix!")
                break
            if object2cam is None:
                print("No object_to_camera matrix!")
                continue
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
        except rospy.ROSInterruptException:
            print("Failed to detect or locate objects!")


if __name__ == '__main__':
    location_publisher()

