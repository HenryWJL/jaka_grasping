#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This is used for grasping and placing objects.
"""

import rospy
import jkrc
import time
from geometry_msgs.msg import TwistStamped
from dh_gripper_msgs.msg import GripperCtrl
from math import pi


def init():
    global joint_start_pose
    # Starting the robot
    robot.login()
    robot.power_on()
    robot.enable_robot()
    # Set the collision level to 1, which means the collision threshold force is 25N
    robot.set_collision_level(1)
    robot.collision_recover()  # If any collision happened, recover the robot from the "collision protection" pattern
    rospy.loginfo("The robot is ready to move")
    # Obtaining the robot's original joints poses
    ret = robot.get_joint_position()
    if ret[0] == 0:
        joint_start_pose = ret[1]

    else:
        rospy.logwarn("Failed to obtain the original pose!")
    # Keep the gripper open
    gripper_control(close=False)


def callback(pose):
    # Obtaining the target pose represented by the robot's joints poses
    global joint_target_pose
    cartesian_target_pose = [pose.twist.linear.x * 1000, pose.twist.linear.y * 1000, 130,
                             3.140934315898051, 0.01111077693116226, 0.7647058015683122]
    ret = robot.kine_inverse(joint_start_pose, cartesian_target_pose)
    if ret[0] == 0:
        joint_target_pose = ret[1]

    else:
        rospy.logwarn("Failed to set the target pose!")
        print("Failed to set the target pose!")


def gripper_control(close=True):
    # Controlling the gripper to open or close
    pub = rospy.Publisher('/gripper/ctrl', GripperCtrl, queue_size=10)
    rate = rospy.Rate(5)
    gripper_ctrl = GripperCtrl()
    gripper_ctrl.force = 25
    gripper_ctrl.speed = 50
    # Closing the gripper
    if close:
        gripper_ctrl.position = 0
    # Opening the gripper
    else:
        gripper_ctrl.position = 1000

    for idx in range(10):
        pub.publish(gripper_ctrl)
        rate.sleep()


def grasp_and_place():
    # Moving to the target pose
    ret = robot.joint_move(joint_target_pose, 0, True, 2)
    time.sleep(1)
    if ret[0] == 0:
        # Moving 20 mm down the z axis
        ret = robot.linear_move([0, 0, -20, 0, 0, 0], 1, True, 10)
        if ret[0] == 0:
            # grasping the object
            gripper_control(close=True)
            # Moving to the placing pose
            ret = robot.joint_move(joint_place_pose, 0, True, 2)
            if ret[0] == 0:
                # placing the object
                gripper_control(close=False)
                # Returning to the original pose
                ret = robot.joint_move(joint_start_pose, 0, True, 2)
                if ret[0] == 0:
                    rospy.loginfo("Back to the original position!")
                    return True

                else:
                    rospy.logerr("Failed to go back to the original position!")
                    print("Failed to go back to the original position!")
                    return False

            else:
                rospy.logerr("Failed to move to the place position!")
                print("Failed to move to the place position!")
                return False

        else:
            rospy.logerr("Failed to move down the gripper!")
            print("Failed to move down the gripper!")
            return False

    else:
        rospy.logerr("Failed to move to the target position!")
        print("Failed to move to the target position!")
        return False


if __name__ == '__main__':
    rospy.init_node('jaka_grasp', anonymous=True)
    rospy.Subscriber('/object_pose', TwistStamped, callback, queue_size=10)
    robot_ip = rospy.get_param("/robot_ip", default="192.168.200.100")
    robot = jkrc.RC(robot_ip)
    # Variables
    joint_start_pose = []  # the original joint position of the robot
    joint_target_pose = []  # the target joint position sightly above the object
    joint_place_pose = [4.64164575143487, 0.06362000542263152, -1.9055709465167618,  # the position where the
                        3.217805620002092, 1.315274659966769, -3.968351295385847]    # robot places the object
    # initializing the robot and the gripper
    init()

    rospy.sleep(1)

    while not rospy.is_shutdown():
        try:
            print("Grasp: g, Exit: e")
            command = str(input())
            if command == 'g':
                if not joint_target_pose:
                    rospy.logwarn("No target position available!")
                    print("No target position available!")

                else:
                    success = grasp_and_place()
                    if not success:
                        res = robot.is_in_collision()
                        collision_state = res[1]
                        if collision_state == 1:
                            # If a collision happened, the JAKA robot will turn into the "collision protection" pattern
                            # and automatically lock itself. In this case, withdrawing from the "collision protection"
                            # pattern and terminating the program.
                            robot.motion_abort()
                            robot.collision_recover()
                            robot.logout()
                            rospy.logerr("A collision happened, please restart the program!")
                        break

            elif command == 'e':
                robot.logout()
                break

            else:
                rospy.logwarn("Invalid option!")

        except rospy.ROSInterruptException:
            rospy.logwarn("No object pose data available!")
