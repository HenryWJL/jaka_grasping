#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This is used for hand-eye calibration. Either ArUco markers or
AprilTag markers are feasible for pose estimation.
"""

import rospy
import numpy as np
import cv2
import transforms3d as tfs
from geometry_msgs.msg import TwistStamped
from tf2_msgs.msg import TFMessage
import time
import yaml
# import jkrc

end_pose = None  # The pose of robot's end
target_pose = None  # The pose of ArUco target
gripper2end = np.identity(4)
gripper2end[2, 3] = 0.1
# joint_position = np.array([[3.645505013109037, 0.18086167531401437, -1.343702557353152, 1.701261909851316, 0.9392576268979733, -4.168356020809264],
#                            [3.640016238212762, 0.180645959270056, -1.4690335788929743, 1.6096664807416525, 0.9896872442855782, -4.168368005033929],
#                            [3.620781557626473, 0.18002277958750956, -1.6250561997982091, 1.4607984419610376, 1.1256962100013408, -4.170261512530897],
#                            [3.6101275818998615, 0.17888427824439582, -1.726442740458651, 1.4930240220834874, 1.1257081942260052, -4.170249528306232],
#                            [3.6712231592387425, 0.1840015421760753, -1.6731608776009292, 1.6940114539293814, 1.1112672035054576, -4.170225559856903],
#                            [3.7059893949900355, 0.18603886036901562, -1.5958146916171836, 1.9102667879976634, 1.111279187730122, -4.170225559856903],
#                            [3.7056418524747694, 0.1855954440564345, -1.722535883218071, 1.8068309449196174, 1.1188652019426584, -4.170225559856903],
#                            [3.705689789373426, 0.18565536517975628, -1.8419466977737022, 1.6941432804006893, 1.243321375081983, -4.1702375440815675],
#                            [3.7061332056860072, 0.18569131785374932, -1.8418028870777299, 1.8604004291692835, 1.1712602321752177, -4.1702375440815675],
#                            [3.7773674370909323, 0.18601489191968695, -1.7410634945491634, 1.8756443629423425, 1.1712602321752177, -4.170225559856903],
#                            [3.776312825320469, 0.18611076571700175, -1.741111431447821, 1.7021247740271497, 1.2353998025788444, -4.170225559856903],
#                            [3.7762289357478185, 0.1860987814923374, -1.796610375868448, 1.6267679693376866, 1.379246451225095, -4.1702375440815675],
#                            [3.773304784929716, 0.1860029076950226, -1.7964066440491542, 1.477516435367812, 1.379246451225095, -4.1702375440815675],
#                            [3.7735924063216606, 0.18669799272555515, -1.6789013212151553, 1.5741212703871763, 1.2417514416509525, -4.170225559856903],
#                            [3.795199963391492, 0.186422355558275, -1.4994735095404357, 1.6278345653328143, 1.1709845950079374, -4.170225559856903],
#                            [3.736213609593538, 0.18365399966080903, -1.5479856509817438, 1.5066141328528655, 1.1726144495622897, -4.1702375440815675],
#                            [3.6124884741587393, 0.15092508210245612, -1.606408746220473, 1.3849982209589935, 1.1726384180116183, -4.170225559856903]])


def get_gripper2base_mat(pose):
    # Calculate the gripper-to-base transformation matrix
    T_end2base = np.array((pose.twist.linear.x, pose.twist.linear.y, pose.twist.linear.z))
    R_end2base = tfs.euler.euler2mat(pose.twist.angular.x, pose.twist.angular.y, pose.twist.angular.z)
    # end2base = np.column_stack((R_end2base, T_end2base))
    # end2base = np.row_stack((end2base, np.array([0, 0, 0, 1])))
    # gripper2base = np.matmul(end2base, gripper2end)
    # return gripper2base[:3, :3], gripper2base[:3, 3]
    return R_end2base, T_end2base


def get_target2cam_mat(pose):
    # Calculate the ArUco-target-to-camera transformation matrix
    pose = pose.transforms[0].transform
    tran = np.array(([pose.translation.x], [pose.translation.y], [pose.translation.z]))
    rot = tfs.quaternions.quat2mat((pose.rotation.w, pose.rotation.x,
                                    pose.rotation.y, pose.rotation.z))
    return rot, tran


def end_pose_callback(pose):
    global end_pose
    if pose is None:
        rospy.logwarn("No robot pose data!")

    else:
        end_pose = pose


def target_pose_callback(pose):
    global target_pose
    if pose is None:
        rospy.logwarn("No ArUco or AprilTag data!")

    else:
        target_pose = pose


if __name__ == '__main__':
    """
    End-pose data and target-pose data are obtained from topics '/robot_driver/tool_point' 
    and '/tf', respectively. 
    """
    rospy.init_node('hand_to_eye_calib', anonymous=True)
    rospy.Subscriber('/robot_driver/tool_point', TwistStamped, end_pose_callback, queue_size=10)
    rospy.Subscriber('/tf', TFMessage, target_pose_callback, queue_size=10)

    R_gripper2base_samples = []
    T_gripper2base_samples = []
    R_target2cam_samples = []
    T_target2cam_samples = []
    sample_number = 0
    cam2base = None

    # robot = jkrc.RC("192.168.200.100")
    # robot.login()
    # robot.enable_robot()

    while not rospy.is_shutdown():
        try:
            # ret = robot.joint_move(joint_position[sample_number, :].tolist(), 0, True, 2)
            # time.sleep(2)
            #
            # (R_target2cam, T_target2cam) = get_target2cam_mat(target_pose)
            # R_target2cam_samples.append(R_target2cam)
            # T_target2cam_samples.append(T_target2cam)
            #
            # (R_gripper2base, T_gripper2base) = get_gripper2base_mat(end_pose)
            # R_gripper2base_samples.append(R_gripper2base)
            # T_gripper2base_samples.append(T_gripper2base)
            #
            # sample_number += 1
            # if sample_number == 17:
            #     R_cam2base, T_cam2base = cv2.calibrateHandEye(R_gripper2base_samples, T_gripper2base_samples,
            #                                                   R_target2cam_samples, T_target2cam_samples,
            #                                                   cv2.CALIB_HAND_EYE_TSAI)
            #     cam2base = np.column_stack((R_cam2base, T_cam2base))
            #     cam2base = np.row_stack((cam2base, np.array([0, 0, 0, 1])))
            #     print(cam2base)  # Transformation matrix
            #     R_cam2base_quaternions = tfs.quaternions.mat2quat(R_cam2base)
            #     cam2base_quaternions = np.concatenate((T_cam2base.reshape(3),
            #                                            R_cam2base_quaternions.reshape(4)), axis=0)
            #     print(cam2base_quaternions.tolist())  # Quaternions
            #     robot.logout()
            #     break

            if end_pose is None:
                rospy.logwarn("Waiting for JAKA data...")
                time.sleep(2)
                continue

            if target_pose is None:
                rospy.logwarn("Waiting for ArUco or AprilTag data...")
                time.sleep(2)
                continue

            print("Record: r, Calculate: c, Save: s, Quit: q")
            command = str(input())

            if command == 'r':
                (R_target2cam, T_target2cam) = get_target2cam_mat(target_pose)
                R_target2cam_samples.append(R_target2cam)
                T_target2cam_samples.append(T_target2cam)

                (R_gripper2base, T_gripper2base) = get_gripper2base_mat(end_pose)
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
                    print(cam2base)  # Transformation matrix
                    R_cam2base_quaternions = tfs.quaternions.mat2quat(R_cam2base)
                    cam2base_quaternions = np.concatenate((T_cam2base.reshape(3),
                                                           R_cam2base_quaternions.reshape(4)), axis=0)
                    print(cam2base_quaternions.tolist())  # Quaternions

            elif command == 's':
                if cam2base is None:
                    rospy.logwarn("No result to save!")

                else:
                    # Modify the following file saving path to your own
                    with open(
                            './jaka_ws/src/jaka_grasping/handeye_calibration/yaml/camera_to_base_matrix_apriltag.yaml',
                            'w', encoding='utf-8') as f:
                        yaml.dump(data=cam2base.tolist(), stream=f)
                    break

            elif command == 'q':
                break

            else:
                rospy.logwarn("Invalid option!")

            time.sleep(2)

        except rospy.ROSInterruptException:
            rospy.logwarn("No available data!")
            continue
