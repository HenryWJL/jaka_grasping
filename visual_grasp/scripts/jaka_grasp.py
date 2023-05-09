#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import jkrc
from geometry_msgs.msg import TwistStamped
import time
from math import pi

robot = jkrc.RC("192.168.200.100")  # Modify the robot ip to your own
joint_start_pose = []  # the original joint position of the robot
joint_target_pose = []  # the target joint position sightly above the object
joint_place_pose = [4.433551674363015, 0.059916880001345894, -1.7483379189204271,   # the position where the
                    3.2127842298677276, 1.5035108767697898, -5.475103893985845]     # robot places the object


def init():
    # Enable the robot and set the robot's starting pose
    global joint_start_pose
    robot.login()
    robot.enable_robot()
    robot.set_collision_level(1)
    rospy.loginfo("The robot is ready to move")
    ret = robot.get_joint_position()
    if ret[0] == 0:
        joint_start_pose = ret[1]

    else:
        rospy.logwarn("Failed to get the original pose!")


def callback(pose):
    # Set the target pose to what is published on the topic 'object_pose'
    global joint_target_pose
    # Obtain the target pose w.r.t the Cartesian space
    tcp_target_pose = [pose.twist.linear.x * 1000, pose.twist.linear.y * 1000, 130,
                       -3.106103956567233, -0.027290779839484594, 2.399677252426657]  # here the unit of x, y, z is mm
    ret = robot.kine_inverse(joint_start_pose, tcp_target_pose)  # Calculate the target pose w.r.t the joint space
    if ret[0] == 0:
        joint_target_pose = ret[1]

    else:
        rospy.logwarn("Failed to set the target pose!")
        print("Failed to set the target pose!")


def grasp_and_place():
    # Perform the grasping and placing tasks
    ret = robot.joint_move(joint_target_pose, 0, True, 2)  # Move to the target position
    time.sleep(1)
    if ret[0] == 0:
        ret = robot.joint_move([0, 0, 0, 0, 0, pi/2], 1, True, 2)  # Adjust the gripper's pose
        time.sleep(1)
        if ret[0] == 0:
            ret = robot.linear_move([0, 0, -20, 0, 0, 0], 1, True, 10)  # Move 20 mm down the z axis
            time.sleep(1)
            if ret[0] == 0:

                # grasp the object

                print("Successful grasp!")
                time.sleep(1)
                ret = robot.joint_move(joint_place_pose, 0, True, 2)  # Move to the placing position
                if ret[0] == 0:

                    # place the object

                    print("Successful place!")
                    time.sleep(1)
                    ret = robot.joint_move(joint_start_pose, 0, True, 2)  # Back to the original position
                    if ret[0] == 0:
                        rospy.loginfo("Back to the original position!")
                        time.sleep(1)
                        return True

                    else:
                        rospy.logwarn("Failed to go back to the original position!")
                        print("Failed to go back to the original position!")
                        return False

                else:
                    rospy.logwarn("Failed to move to the place position!")
                    print("Failed to move to the place position!")
                    return False

            else:
                rospy.logwarn("Failed to move down the gripper!")
                print("Failed to move down the gripper!")
                return False

        else:
            rospy.logwarn("Failed to adjust the gripper's pose!")
            print("Failed to adjust the gripper's pose!")
            return False

    else:
        rospy.logwarn("Failed to move to the target position!")
        print("Failed to move to the target position!")
        return False


if __name__ == '__main__':
    rospy.init_node('jaka_grasp', anonymous=True)
    rospy.Subscriber('/object_pose', TwistStamped, callback, queue_size=10)
    init()
    while not rospy.is_shutdown():
        try:
            print("Grasp: g, Exit: e")
            command = str(input())
            if command == 'g':
                if not joint_target_pose:
                    rospy.logwarn("No target position available!")

                else:
                    success = grasp_and_place()
                    if not success:
                        res = robot.is_in_collision()
                        collision_state = res[1]

                        if collision_state == 1:
                            """
                            If a collision happens, the JAKA robot will turn into the "collision protection" pattern
                            and automatically lock itself. In this case, withdraw from the "collision protection" 
                            pattern and terminate the program. 
                            """
                            robot.motion_abort()
                            time.sleep(3)
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
