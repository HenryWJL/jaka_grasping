import jkrc
import time

PI = 3.14159265
start_pose = [-0.2664363286039594, -0.015709620237819116, 0.3284071097998305,
              0.5166579082169311, 1.6366877768864314, -0.5233736902150538]
# joint_pose=[PI/2,PI/3,0,PI/4,0,0]
robot = jkrc.RC("192.168.200.100")
robot.login()
robot.power_on()
robot.enable_robot()
robot.collision_recover()
ret = robot.get_tcp_position()
# if ret[0] == 0:
#     start_pose = ret[1]
#     print(start_pose)
# target_pose = [start_pose[0], start_pose[1], start_pose[2]-0.05, start_pose[3], start_pose[4], start_pose[5]]
target_pose = [0, 0, -10, 0, 0, 0]
# robot.joint_move(joint_pose, 0, True, 1)
robot.linear_move(target_pose, 1, True, 5)
time.sleep(3)
# robot.set_torsenosr_brand(2)
# robot.set_torque_sensor_mode(1)
# robot.set_compliant_type(1, 1)
# print("inint sensor comple")
# print("ready to run")
# ret = robot.get_joint_position()
# joint_pos_origin = ret[1]
# joint_pos = ret[1]
# print(joint_pos)
# joint_pos[3] += PI / 4
# if joint_pos[3] > 265 * PI / 180:
#     joint_pos[3] -= PI / 2
#     joint_pos[4] += PI / 4
# if joint_pos[4] > 320 * PI / 180:
#     joint_pos[4] -= PI / 2
#     joint_pos[5] += PI / 4
# if joint_pos[5] > 265 * PI / 180:
#     joint_pos[5] -= PI
# print(joint_pos)
# ret = robot.start_torq_sensor_payload_identify(joint_pos)
# time.sleep(1)
# flag = 1
# while 1 == flag:
#     ret = robot.get_torq_sensor_identify_staus()
#     print(ret)
#     time.sleep(1)
#     flag = ret[1]
# print("identy_finish")
# ret = robot.get_torq_sensor_payload_identify_result()
# print(ret)
# robot.joint_move(joint_pos_origin, 0, 1, 10)
# print("back")
robot.logout()  # 登出
