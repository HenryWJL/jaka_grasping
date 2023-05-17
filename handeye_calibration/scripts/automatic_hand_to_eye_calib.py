#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This is used for hand-eye calibration without human intervention.
Either ArUco or AprilTag markers is feasible for pose estimation.
"""

import rospy
import numpy as np
import cv2
import transforms3d as tfs
from tf2_msgs.msg import TFMessage
import time
import yaml
import jkrc

target_pose = None  # The pose of ArUco target
# The 17 positions where samples are recorded
joint_position = np.array([[3.645505013109037, 0.18086167531401437, -1.343702557353152,
                            1.701261909851316, 0.9392576268979733, -4.168356020809264],
                           [3.640016238212762, 0.180645959270056, -1.4690335788929743,
                            1.6096664807416525, 0.9896872442855782, -4.168368005033929],
                           [3.620781557626473, 0.18002277958750956, -1.6250561997982091,
                            1.4607984419610376, 1.1256962100013408, -4.170261512530897],
                           [3.6101275818998615, 0.17888427824439582, -1.726442740458651,
                            1.4930240220834874, 1.1257081942260052, -4.170249528306232],
                           [3.6712231592387425, 0.1840015421760753, -1.6731608776009292,
                            1.6940114539293814, 1.1112672035054576, -4.170225559856903],
                           [3.7059893949900355, 0.18603886036901562, -1.5958146916171836,
                            1.9102667879976634, 1.111279187730122, -4.170225559856903],
                           [3.7056418524747694, 0.1855954440564345, -1.722535883218071,
                            1.8068309449196174, 1.1188652019426584, -4.170225559856903],
                           [3.705689789373426, 0.18565536517975628, -1.8419466977737022,
                            1.6941432804006893, 1.243321375081983, -4.1702375440815675],
                           [3.7220242875909424, 0.08424485606998611, -1.8235509129139176,
                            1.7100703149796168, 1.1726384180116183, -3.8247802839068767],
                           [3.7198671271513586, 0.08424485606998611, -1.8235748813632464,
                            1.7168054492409845, 1.3845195100774108, -3.8247922681315414],
                           [3.776312825320469, 0.18611076571700175, -1.741111431447821,
                            1.7021247740271497, 1.2353998025788444, -4.170225559856903],
                           [3.7762289357478185, 0.1860987814923374, -1.796610375868448,
                            1.6267679693376866, 1.379246451225095, -4.1702375440815675],
                           [3.773304784929716, 0.1860029076950226, -1.7964066440491542,
                            1.477516435367812, 1.379246451225095, -4.1702375440815675],
                           [3.7735924063216606, 0.18669799272555515, -1.6789013212151553,
                            1.5741212703871763, 1.2417514416509525, -4.170225559856903],
                           [3.795199963391492, 0.186422355558275, -1.4994735095404357,
                            1.6278345653328143, 1.1709845950079374, -4.170225559856903],
                           [3.736213609593538, 0.18365399966080903, -1.5479856509817438,
                            1.5066141328528655, 1.1726144495622897, -4.1702375440815675],
                           [3.6124884741587393, 0.15092508210245612, -1.606408746220473,
                            1.3849982209589935, 1.1726384180116183, -4.170225559856903]])


def get_end2base_mat(pose):
    # Calculate the gripper-to-base transformation matrix
    tran = np.array((pose[0], pose[1], pose[2]))
    rot = tfs.euler.euler2mat(pose[3], pose[4], pose[5])
    return rot, tran


def get_target2cam_mat(pose):
    # Calculate the ArUco-target-to-camera transformation matrix
    pose = pose.transforms[0].transform
    tran = np.array(([pose.translation.x], [pose.translation.y], [pose.translation.z]))
    rot = tfs.quaternions.quat2mat((pose.rotation.w, pose.rotation.x,
                                    pose.rotation.y, pose.rotation.z))
    return rot, tran


def callback(pose):
    global target_pose
    if pose is None:
        rospy.logwarn("No ArUco or AprilTag data!")

    else:
        target_pose = pose


if __name__ == '__main__':
    """
    Target-pose data are obtained from topic '/tf' while end-pose data are
    obtained from a function. 
    """
    rospy.init_node('automatic_hand_to_eye_calib', anonymous=True)
    rospy.Subscriber('/tf', TFMessage, callback, queue_size=10)

    robot = jkrc.RC("192.168.200.100")
    robot.login()
    robot.enable_robot()

    R_end2base_samples = []
    T_end2base_samples = []
    R_target2cam_samples = []
    T_target2cam_samples = []
    sample_number = 0
    cam2base = None

    while not rospy.is_shutdown():
        try:
            ret = robot.joint_move(joint_position[sample_number, :].tolist(), 0, True, 2)
            time.sleep(1)
            if ret[0] == 0:
                ret = robot.get_tcp_position()
                if ret[0] == 0:
                    end_pose = ret[1]
                    (R_target2cam, T_target2cam) = get_target2cam_mat(target_pose)
                    R_target2cam_samples.append(R_target2cam)
                    T_target2cam_samples.append(T_target2cam)

                    (R_end2base, T_end2base) = get_end2base_mat(end_pose)
                    R_end2base_samples.append(R_end2base)
                    T_end2base_samples.append(T_end2base)

                    sample_number += 1

                else:
                    rospy.logwarn("Failed to get the end pose!")
                    print("Failed to get the end pose!")
                    continue

            else:
                rospy.logerr("Failed to move to the target position!")
                print("Failed to move to the target position!")
                continue

            if sample_number == 17:
                R_cam2base, T_cam2base = cv2.calibrateHandEye(R_end2base_samples, T_end2base_samples,
                                                              R_target2cam_samples, T_target2cam_samples,
                                                              cv2.CALIB_HAND_EYE_TSAI)
                cam2base = np.column_stack((R_cam2base, T_cam2base))
                cam2base = np.row_stack((cam2base, np.array([0, 0, 0, 1])))
                print(cam2base)  # Transformation matrix
                R_cam2base_quaternions = tfs.quaternions.mat2quat(R_cam2base)
                cam2base_quaternions = np.concatenate((T_cam2base.reshape(3),
                                                       R_cam2base_quaternions.reshape(4)), axis=0)
                print(cam2base_quaternions.tolist())  # Quaternions
                with open('./jaka_ws/src/jaka_grasping/handeye_calibration/yaml/camera_to_base_matrix_apriltag.yaml',
                          'w', encoding='utf-8') as f:
                    yaml.dump(data=cam2base.tolist(), stream=f)
                robot.logout()
                break
        except rospy.ROSInterruptException:
            rospy.logwarn("No available data!")
            continue
