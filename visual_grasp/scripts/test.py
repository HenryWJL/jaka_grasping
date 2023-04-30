import jkrc
import time

PI = 3.14159265
# target_pose = [-266.4363286039594, -15.709620237819116, 328.4071097998305,
#                516.6579082169311, 636.6877768864314, -523.3736902150538]
robot = jkrc.RC("192.168.200.100")
robot.login()
robot.power_on()
robot.enable_robot()
# robot.collision_recover()
# target_pose = [start_pose[0], start_pose[1], start_pose[2]-0.05, start_pose[3], start_pose[4], start_pose[5]]

# robot.joint_move(joint_pose, 0, True, 1)
# robot.jog(2, 0, 0, 5, 300)
# robot.jog(2, 0, 0, 5, 300)
# robot.jog(2, 0, 0, 5, 300)
# robot.jog(2, 0, 0, 5, 300)
# robot.jog(2, 0, 0, 5, 300)
# robot.jog(2, 0, 0, 5, 300)
ret = robot.get_tcp_position()
target_pose = ret[1]
print(target_pose)
# ret = robot.get_joint_position()
# print(ret[1])
# ref_pos = ret[1]
# ret = robot.kine_inverse(ref_pos, target_pose)
# print(ret[1])
# robot.joint_move(ret[1], 0, True, 3)
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
