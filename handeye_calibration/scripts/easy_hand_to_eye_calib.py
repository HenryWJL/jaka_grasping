#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This is an easier practice of hand-eye calibration. 
"""

import rospy
import numpy as np
import transforms3d as tfs
from tf2_msgs.msg import TFMessage
import time
import yaml

upper2base = np.identity(4)
upper2base[2, 3] = 0.2820869032411154
cam2base = None


def callback(pose):
    # Calculate the camera-to-base transformation matrix directly from the ArUco target's pose
    global cam2base
    pose = pose.transforms[0].transform
    tran = np.array(([pose.translation.x], [pose.translation.y], [pose.translation.z]))
    rot = tfs.quaternions.quat2mat((pose.rotation.w, pose.rotation.x,
                                    pose.rotation.y, pose.rotation.z))
    mat = np.column_stack((rot, tran))
    mat = np.row_stack((mat, np.array([0, 0, 0, 1])))
    cam2upper = np.linalg.inv(mat)
    cam2base = np.matmul(upper2base, cam2upper)


if __name__ == '__main__':
    # Obtain pose data from topic '/tf'
    rospy.init_node('easy_hand_to_eye_calib', anonymous=True)
    rospy.Subscriber('/tf', TFMessage, callback, queue_size=10)
    while not rospy.is_shutdown():
        if cam2base is None:
            rospy.logwarn("Waiting for ArUco data...")
            time.sleep(2)
            continue
        print("Enter 'r' to record the result:")
        command = str(input())
        if command == 'r':
            # Modify the following file saving path to your own
            R_cam2base_quaternions = tfs.quaternions.mat2quat(cam2base[:3, :3])
            cam2base_quaternions = np.concatenate((cam2base[:3, 3].reshape(3),
                                                   R_cam2base_quaternions.reshape(4)), axis=0)
            print(cam2base_quaternions.tolist())  # Quaternions
            with open('./jaka_ws/src/jaka_grasping/handeye_calibration/yaml/camera_to_base_matrix_aruco.yaml',
                      'w', encoding='utf-8') as f:
                yaml.dump(data=cam2base.tolist(), stream=f)
            break

        else:
            rospy.logwarn("Invalid option!")
            time.sleep(2)
            continue
