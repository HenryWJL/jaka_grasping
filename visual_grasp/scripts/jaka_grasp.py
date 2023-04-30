#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import jkrc
from geometry_msgs.msg import TwistStamped
import time

robot = jkrc.RC("192.168.200.100")
joint_start_pose = []
joint_target_pose = []
joint_place_pose = [0.1779175433766374, -0.5681403819834954, 1.7409050447804615,
                    2.5849426839268452, -0.7791124108252016, -1.8307251420048847]


# the position where robotic arm places the object


def init():
    global joint_start_pose
    robot.login()
    robot.enable_robot()
    robot.set_collision_level(1)
    rospy.loginfo("The robotic arm is ready to move")
    ret = robot.get_joint_position()
    if ret[0] == 0:
        joint_start_pose = ret[1]

    else:
        rospy.logwarn("Failed to get the original pose!")


def grasp_and_place():
    ret = robot.joint_move(joint_target_pose, 0, True, 2)
    if ret[0] == 0:

        # grasp the object

        print("Successful grasp!")
        time.sleep(3)
        ret = robot.joint_move(joint_place_pose, 0, True, 2)
        if ret[0] == 0:

            # place the object

            print("Successful place!")
            time.sleep(3)
            ret = robot.joint_move(joint_start_pose, 0, True, 2)
            if ret[0] == 0:
                rospy.loginfo("Back to the original position!")
                time.sleep(3)
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
        rospy.logwarn("Failed to move to the target position!")
        print("Failed to move to the target position!")
        return False


def callback(pose):
    global joint_target_pose
    tcp_target_pose = [pose.twist.linear.x * 1000, pose.twist.linear.y * 1000, pose.twist.linear.z * 1000 - 400,
                       pose.twist.angular.x, pose.twist.angular.y, pose.twist.angular.z]
    ret = robot.kine_inverse(joint_start_pose, tcp_target_pose)
    if ret[0] == 0:
        joint_target_pose = ret[1]

    else:
        rospy.logwarn("Failed to set the target pose!")


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
                    rospy.logwarn("No target position is available!")

                else:
                    success = grasp_and_place()
                    if not success:
                        res = robot.is_in_collision()
                        collision_state = res[1]
                        if collision_state == 1:
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
            rospy.logwarn("Failed!")
