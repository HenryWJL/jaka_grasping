import rospy
import numpy as np
import transforms3d as tfs
from tf2_msgs.msg import TFMessage
import time
import yaml

cam2base = None


def callback(pose):
    global cam2base
    pose = pose.transforms[0].transform
    tran = np.array(([pose.translation.x], [pose.translation.y], [pose.translation.z]))
    rot = tfs.quaternions.quat2mat((pose.rotation.w, pose.rotation.x,
                                    pose.rotation.y, pose.rotation.z))
    mat = np.column_stack((rot, tran))
    mat = np.row_stack((mat, np.array([0, 0, 0, 1])))
    cam2upper = np.linalg.inv(mat)
    upper2base = np.identity(4)
    up2base[2, 3] = 0.2820869032411154
    cam2base = np.matmul(upper2base, cam2upper)


if __name__ == '__main__':
    rospy.init_node('easy_hand_to_eye_calib', anonymous=True)
    rospy.Subscriber('/tf', TFMessage, callback, queue_size=10)
    while not rospy.is_shutdown():
        if cam2base is None:
            rospy.logwarn("Waiting for ArUco data...")
            time.sleep(2)
            continue
        print("Enter 'r' to record the result:")
        command = str(input())
        if command == 'r':
            with open('./jaka_ws/src/jaka_grasping/handeye_calibration/yaml/camera_to_base_matrix.yaml',
                      'w', encoding='utf-8') as f:
                yaml.dump(data=cam2base, stream=f)

        else:
            rospy.logwarn("Invalid option!")
            time.sleep(2)
            continue
