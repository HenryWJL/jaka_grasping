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
    tran = np.array((pose.twist.linear.x, pose.twist.linear.y, pose.twist.linear.z))
    rot = tfs.euler.euler2mat(pose.twist.angular.x, pose.twist.angular.y, pose.twist.angular.z)
    return rot, tran


# def get_target2cam_mat(pose):  # using topic
#     tran = np.array([[pose.pose.position.x], [pose.pose.position.y], [pose.pose.position.z]])
#     rot = tfs.quaternions.quat2mat((pose.pose.orientation.w, pose.pose.orientation.x,
#                                     pose.pose.orientation.y, pose.pose.orientation.z))
#     return tran, rot


def get_target2cam_mat():  # using tf transform
    listener = tf.TransformListener()
    tran, rot = listener.lookupTransform('/camera_color_optical_frame', '/camera_link', rospy.Time(0))
    tran = np.array((tran[0], tran[1], tran[2]))                        # '/camera_link' is the frame of ArUco
    rot = tfs.quaternions.quat2mat((rot[0], rot[1], rot[2], rot[3]))
    return rot, tran


def callback(pose):
    global tcp_pose
    if pose is None:
        rospy.logwarn("No tcp pose data!")
    else:
        tcp_pose = pose


if __name__ == '__main__':
    rospy.init_node('hand_to_eye_calib', anonymous=True)
    rospy.Subscriber('/robot_driver/tool_point', TwistStamped, callback, queue_size=10)
    R_gripper2base_samples = []
    T_gripper2base_samples = []
    R_target2cam_samples = []
    T_target2cam_samples = []
    sample_number = 0
    cam2base = None
    while not rospy.is_shutdown():
        try:
            print("Record: r, Calibrate: c, Save: s, Quit: q")
            command = str(input())

            if command == 'r':
                (R_target2cam, T_target2cam) = get_target2cam_mat()
                R_target2cam_samples.append(R_target2cam)
                T_target2cam_samples.append(T_target2cam)
                (R_gripper2base, T_gripper2base) = get_gripper2base_mat(tcp_pose)
                R_gripper2base_samples.append(R_gripper2base)
                T_gripper2base_samples.append(T_gripper2base)
                sample_number += 1
                print(f"{sample_number} samples have been recorded")

            elif command == 'c':
                if sample_number < 3:
                    rospy.logwarn("No enough samples!")

                else:
                    R_cam2base, T_cam2base = cv2.calibrateHandEye(R_gripper2base_samples, T_gripper2base_samples,
                                                                  R_target2cam_samples, T_target2cam_samples,
                                                                  cv2.CALIB_HAND_EYE_TSAI)
                    cam2base = np.column_stack((R_cam2base, T_cam2base))
                    cam2base = np.row_stack((cam2base, np.array([0, 0, 0, 1])))
                    print(cam2base)

            elif command == 's':
                if cam2base is None:
                    rospy.logwarn("No result to save!")

                else:
                    cam2base = cam2base.tolist()  # change np.array to list so that it can be stored in .yaml file
                    with open('./jaka_ws/src/jaka_grasping/handeye_calibration/yaml/camera_to_base_matrix.yaml',
                              'w', encoding='utf-8') as f:
                        yaml.dump(data=cam2base, stream=f)
                    break

            elif command == 'q':
                break

            else:
                rospy.logwarn("Invalid option!")

            time.sleep(2)

        except rospy.ROSInterruptException:
            rospy.logwarn("Waiting for robot data!")
            continue

        except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Waiting for ArUco data!")
            continue
