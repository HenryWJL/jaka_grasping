#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import jkrc
from geometry_msgs.msg import TwistStamped
import time

robot = jkrc.RC("192.168.200.100")
target_pose = None


def init_robot():
    robot.login()
    robot.power_on()
    robot.enable_robot()
    print("The robotic arm is ready to move")


def move_to_grasp():
    ret = robot.linear_move(target_pose, 0, True, 1)
    if ret == 0:
        print("Successful movement!")
    else:
        print("Failed to move to the target position!")
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
    while not rospy.is_shutdown():
        try:
            rospy.Subscriber('/object_pose', TwistStamped, callback, queue_size=10)
            command = str(input("Grasp: g, Reset: r"))
            if command == 'g':
                if target_pose is None:
                    print("No target position is available!")
                else:
                    move_to_grasp()
            elif command == 'r':
                pass
            else:
                print("Invalid option!")
                    
        except rospy.ROSInterruptException:
            print("Failed!")
