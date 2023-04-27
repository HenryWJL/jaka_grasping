#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2
import transforms3d as tfs
import tf
from geometry_msgs.msg import TwistStamped
import time
import yaml

tcp_pose = TwistStamped()


def get_gripper2base_mat(pose):
    tran = np.array([[pose.twist.linear.x], [pose.twist.linear.y], [pose.twist.linear.z]])
    tran = tran.reshape((3, 1))
    rot = tfs.euler.euler2mat(pose.twist.angular.x, pose.twist.angular.y, pose.twist.angular.z)
    mat = np.column_stack(rot, tran)
    mat = np.row_stack((mat, np.array([0, 0, 0, 1])))
    return mat


# def get_target2cam_mat(pose):  # using topic
#     tran = np.array([[pose.pose.position.x], [pose.pose.position.y], [pose.pose.position.z]])
#     rot = tfs.quaternions.quat2mat([pose.pose.orientation.w, pose.pose.orientation.x,
#                                     pose.pose.orientation.y, pose.pose.orientation.z])
#     return tran, rot


def get_target2cam_mat():  # using tf transform
    listener = tf.TransformListener()
    tran, rot = listener.lookupTransform('/camera_color_optical_frame', '/camera_link', rospy.Time(0))
    tran = np.array(tran)                                      # '/camera_link' is the frame of ArUco
    tran = tran.reshape((3, 1))
    rot = np.array(rot)
    mat = np.column_stack(rot, tran)
    mat = np.row_stack((mat, np.array([0, 0, 0, 1])))
    return mat


def callback(pose):
    global tcp_pose
    if pose is None:
        raise IOException()
    else:
        tcp_pose = pose


if __name__ == '__main__':
    rospy.init_node('hand_to_eye_calib', anonymous=True)
    R_gripper2base_samples = []
    T_gripper2base_samples = []
    R_target2cam_samples = []
    T_target2cam_samples = []
    sample_number = 0
    cam2base = None
    while not rospy.is_shutdown():
        try:
            rospy.Subscriber('/robot_driver/tool_point', TwistStamped, callback, queue_size=10)
            print("Record: r, Calibrate: c, Save: s")
            command = str(input())

            if command == 'r':
                gripper2base = get_gripper2base_mat(tcp_pose)
                R_gripper2base_samples.append(gripper2base[:3, :3])
                T_gripper2base_samples.append(gripper2base[:3, 3])
                target2cam = get_target2cam_mat()
                R_target2cam_samples.append(target2cam[:3, :3])
                T_target2cam_samples.append(target2cam[:3, 3])
                sample_number += 1
                print(f"{sample_number} samples have been recorded")

            elif command == 'c':
                if sample_number == 0:
                    print("No sample is available!")

                else:
                    R_cam2base, T_cam2base = cv2.calibrateHandEye(R_gripper2base_samples, T_gripper2base_samples,
                                                                  R_target2cam_samples, T_target2cam_samples,
                                                                  cv2.CALIB_HAND_EYE_TSAI)
                    cam2base = np.column_stack((R_cam2base, T_cam2base))
                    cam2base = np.row_stack((cam2base, np.array([0, 0, 0, 1])))
                    print(cam2base)

            elif command == 's':
                if cam2base is None:
                    print("No result to save!")

                else:
                    cam2base = cam2base.tolist()  # change np.array to list so that it can be stored in .yaml file
                    with open('~/handeye_calibration/yaml/camera_to_base_matrix.yaml', 'w', encoding='utf-8') as f:
                        yaml.dump(data=cam2base, stream=f)

            else:
                print("Invalid option!")

            time.sleep(2)

        except rospy.ROSInterruptException:
            rospy.loginfo("Waiting for robot data!")
            continue

        except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("Waiting for ArUco data!")
            continue
