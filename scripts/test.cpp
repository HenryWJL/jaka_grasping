bool panel_control_callback(sensor_msgs::JointState &joint_states)
{   
    JointValue joint_pose;
    joint_pose.jVal[0] = joint_states.position[0];
    joint_pose.jVal[1] = joint_states.position[1];
    joint_pose.jVal[2] = joint_states.position[2];
    joint_pose.jVal[3] = joint_states.position[3];
    joint_pose.jVal[4] = joint_states.position[4];
    joint_pose.jVal[5] = joint_states.position[5];

    OptionalCond* p = nullptr;

    int sdk_res = robot.joint_move(&joint_pose, MoveMode::ABS, is_block, 0.5, 0.5, 0.2, p);

    switch(sdk_res)
    {
        case 0:
            res.ret = 1;
            res.message = "cmd has been executed!";
            break;
        default:
            res.ret = sdk_res;
            res.message = mapErr[sdk_res]+"\n";
            break;
    }
    return true;
}
