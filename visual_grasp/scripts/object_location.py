# -*- coding: utf-8 -*-
# ! /usr/bin/env python3

import rospy
import numpy as np
import yaml

if __name__ == '__main__':
    rospy.init_node('object_location', anonymous=True)
    # paste the codes in 'object_recognition' here

    T_object2cam = None
    R_object2cam = None
    object2cam = np.column_stack((R_object2cam, T_object2cam))
    object2cam = np.row_stack((object2cam, np.array([0, 0, 0, 1])))
    with open('~/handeye_calibration/yaml/camera_to_base_matrix.yaml', 'r') as f:
        cam2base = yaml.load(f.read(), Loader=yaml.FullLoader)
        cam2base = np.array(cam2base)
    object2base = np.matmul(object2cam, cam2base)

