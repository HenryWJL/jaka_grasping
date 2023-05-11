import rospy
import jkrc
import time
import transforms3d as tfs
from math import pi
import numpy as np
import tf
from dh_gripper_msgs.msg import GripperCtrl

# target_pose = [-266.4363286039594, -15.709620237819116, 328.4071097998305,
#                516.6579082169311, 636.6877768864314, -523.3736902150538]
robot = jkrc.RC("192.168.200.100")
robot.login()
robot.power_on()
robot.enable_robot()
# robot.collision_recover()
# target_pose = [start_pose[0], start_pose[1], start_pose[2]-0.05, start_pose[3], start_pose[4], start_pose[5]]
# rot = tfs.quaternions.quat2mat((0.9445453031904033, -0.32583345460238583, 0.03934682226708879, -0.01088841863202325))
# tran = np.array((0.3565662205219269, -0.22302578389644623, 0.5799561142921448))
# mat = np.column_stack((rot, tran))
# mat = np.row_stack((mat, np.array([0, 0, 0, 1])))
# cam2up = np.linalg.inv(mat)
# up2base = np.identity(4)
# up2base[2, 3] = 0.2820869032411154
# cam2base = np.matmul(up2base, cam2up)
# print(cam2base)
# ret = robot.joint_move([0, 0, 0, 0, 0, -pi], 1, True, 2)
while True:
    ret = robot.get_joint_position()
    target_pose = ret[1]
    print(target_pose)
    time.sleep(3)
# ret = robot.get_joint_position()
# print(ret[1])
# ref_pos = ret[1]
# ret = robot.kine_inverse(ref_pos, target_pose)
# print(ret[1])
# robot.joint_move(ret[1], 0, True, 3)

robot.logout()  # 登出

