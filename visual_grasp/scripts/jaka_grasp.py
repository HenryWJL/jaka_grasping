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

robot = jkrc.RC("192.168.200.100")  # Modify the robot ip to your own
joint_start_pose = []  # the original joint position of the robot
joint_target_pose = []  # the target joint position sightly above the object
joint_place_pose = [4.64164575143487, 0.06362000542263152, -1.9055709465167618,   # the position where the
                    3.217805620002092, 1.315274659966769, -3.968351295385847]     # robot places the object
gripper_close = GripperCtrl()  # Control the gripper to close
gripper_close.position = 0
gripper_close.force = 25
gripper_close.speed = 50
gripper_open = GripperCtrl()  # Control the gripper to open
gripper_open.position = 1000
gripper_open.force = 25
gripper_open.speed = 50


def init(publisher, publish_rate):
    # Enable the robot and set the robot's starting pose
    global joint_start_pose
    robot.login()
    robot.enable_robot()
    robot.set_collision_level(1)  # Set the collision level to 1, which means the collision threshold force is 25N
    rospy.loginfo("The robot is ready to move")
    ret = robot.get_joint_position()
    if ret[0] == 0:
        joint_start_pose = ret[1]

    else:
        rospy.logwarn("Failed to get the original pose!")
    for idx in range(10):  # Keep the gripper open
        publisher.publish(gripper_open)
        publish_rate.sleep()


def callback(pose):
    # Set the target pose to what is published on the topic 'object_pose'
    global joint_target_pose
    # Obtain the target pose w.r.t the Cartesian space
    tcp_target_pose = [pose.twist.linear.x * 1000, pose.twist.linear.y * 1000, 130,
                       3.140934315898051, 0.01111077693116226, 0.7647058015683122]  # here the unit of x, y, z is mm
    ret = robot.kine_inverse(joint_start_pose, tcp_target_pose)  # Calculate the target pose w.r.t the joint space
    if ret[0] == 0:
        joint_target_pose = ret[1]

    else:
        rospy.logwarn("Failed to set the target pose!")
        print("Failed to set the target pose!")


def grasp_and_place(publisher, publish_rate):
    # Perform the grasping and placing tasks
    ret = robot.joint_move(joint_target_pose, 0, True, 2)  # Move to the target position
    time.sleep(2)
    if ret[0] == 0:
        ret = robot.linear_move([0, 0, -20, 0, 0, 0], 1, True, 10)  # Move 20 mm down the z axis
        if ret[0] == 0:
            for idx in range(10):  # grasp the object
                publisher.publish(gripper_close)
                publish_rate.sleep()
            print("Successful grasp!")
            ret = robot.joint_move(joint_place_pose, 0, True, 2)  # Move to the placing position
            if ret[0] == 0:
                for idx in range(10):  # place the object
                    publisher.publish(gripper_open)
                    publish_rate.sleep()
                print("Successful place!")
                ret = robot.joint_move(joint_start_pose, 0, True, 2)  # Back to the original position
                if ret[0] == 0:
                    rospy.loginfo("Back to the original position!")
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
        rospy.logwarn("Failed to move to the target position!")
        print("Failed to move to the target position!")
        return False


if __name__ == '__main__':
    rospy.init_node('jaka_grasp', anonymous=True)
    rospy.Subscriber('/object_pose', TwistStamped, callback, queue_size=10)
    pub = rospy.Publisher('/gripper/ctrl', GripperCtrl, queue_size=10)
    rate = rospy.Rate(5)
    init(pub, rate)
    while not rospy.is_shutdown():
        try:
            print("Grasp: g, Exit: e")
            command = str(input())
            if command == 'g':
                if not joint_target_pose:
                    rospy.logwarn("No target position available!")
                    print("No target position available!")

                else:
                    success = grasp_and_place(pub, rate)
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
