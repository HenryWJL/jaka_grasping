void panel_control_callback(const sensor_msgs::JointState& joint_states)
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

}

ros::Subscriber sub = n.subscribe("jog_panel_command", 10, panel_control_callback);
