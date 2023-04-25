#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import jkrc
from geometry_msgs.msg import TwistStamped
import time

robot = jkrc.RC("192.168.200.100")
start_pose = []
target_pose = []
place_pose = []  # the position where robotic arm places the object


def init():
    global start_pose
    robot.login()
    robot.power_on()
    robot.enable_robot()
    print("The robotic arm is ready to move")
    ret = robot.get_tcp_position()
    if ret[0] == 0:
        start_pose = list(ret[1])

    else:
        print("Failed to get the original pose!")


def move_to_grasp():
    ret = robot.linear_move(target_pose, 0, True, 1)
    if ret[0] == 0:

        # grasp the object

        print("Successful grasp!")

    else:
        print("Failed to move to the target position!")
    time.sleep(3)
    ret = robot.linear_move(place_pose, 0, True, 1)
    if ret[0] == 0:

        # place the object

        print("Successful place!")

    else:
        print("Failed to move to the place position!")
    time.sleep(3)
    ret = robot.linear_move(start_pose, 0, True, 1)
    if ret[0] == 0:
        print("Back to the original position!")

    else:
        print("Failed to go back to the original position!")
    time.sleep(3)


def move_to_place():
    pass


def reset():
    pass


def callback(pose):
    global target_pose
    target_pose = [pose.twist.linear.x, pose.twist.linear.y, pose.twist.linear.z,
                   pose.twist.angular.x, pose.twist.angular.y, pose.twist.angular.z]


if __name__ == '__main__':
    rospy.init_node('jaka_grasp', anonymous=True)
    init()
    while not rospy.is_shutdown():
        try:
            rospy.Subscriber('/object_pose', TwistStamped, callback, queue_size=10)
            command = str(input("Grasp: g, E: e"))
            if command == 'g':
                if not target_pose:
                    print("No target position is available!")

                else:
                    move_to_grasp()

            elif command == 'e':
                robot.disable_robot()
                robot.power_off()
                robot.logout()
                break

            else:
                print("Invalid option!")
                    
        except rospy.ROSInterruptException:
            print("Failed!")
