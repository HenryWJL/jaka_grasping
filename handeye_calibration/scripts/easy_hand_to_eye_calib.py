#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import transforms3d as tfs
from tf2_msgs.msg import TFMessage
import time
import yaml

upper2base = np.identity(4)
upper2base[2, 3] = 0.2820869032411154
right2left = np.identity(4)
right2left[1, 1] = -1
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
    """ 
    All the frames in ROS are right handed, but the frame of JAKA base_link is left-handed. Therefore, a 
    transformation from right-handed frame to left-handed frame is necessary.
    """
    cam2base = np.matmul(right2left, cam2base)
    cam2base = np.matmul(cam2base, right2left)


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
            with open('./jaka_ws/src/jaka_grasping/handeye_calibration/yaml/camera_to_base_matrix.yaml',
                      'w', encoding='utf-8') as f:
                yaml.dump(data=cam2base.tolist(), stream=f)
            break

        else:
            rospy.logwarn("Invalid option!")
            time.sleep(2)
            continue
