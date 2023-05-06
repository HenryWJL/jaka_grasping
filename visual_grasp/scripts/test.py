import jkrc
import time
import transforms3d as tfs
from math import pi
import numpy as np
import tf

# target_pose = [-266.4363286039594, -15.709620237819116, 328.4071097998305,
#                516.6579082169311, 636.6877768864314, -523.3736902150538]
robot = jkrc.RC("192.168.200.100")
robot.login()
robot.power_on()
robot.enable_robot()
# robot.collision_recover()
# target_pose = [start_pose[0], start_pose[1], start_pose[2]-0.05, start_pose[3], start_pose[4], start_pose[5]]
# rot = tfs.quaternions.quat2mat((0.9501563109375586, -0.3116919399268672, -0.006500308387740517, 0.0029774756119906576))
# tran = np.array((0.23701876401901245, -0.16576284170150757, 0.8335840106010437))
# mat = np.column_stack((rot, tran))
# mat = np.row_stack((mat, np.array([0, 0, 0, 1])))
# print(np.linalg.inv(mat))
while True:
    ret = robot.get_tcp_position()
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

