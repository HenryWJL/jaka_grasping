#include "ros/ros.h"
#include "std_msgs/String.h"

#include "Eigen/Dense"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/StdVector"

#include "robot_msgs/Move.h"
#include "std_srvs/Empty.h"
#include "std_srvs/SetBool.h"
#include "robot_msgs/RobotMsg.h"
#include "robot_msgs/SetUserFrame.h"
#include "robot_msgs/SetTcp.h"
#include "robot_msgs/SetLoad.h"
#include "robot_msgs/ServoL.h"
#include "robot_msgs/ClearErr.h"
#include "robot_msgs/SetCollision.h"
#include "robot_msgs/SetAxis.h"

#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"

#include <pthread.h>
#include <unistd.h>
#include <sstream>

#include <vector>

#include "libs/robot.h"
#include "libs/conversion.h"
#include "time.h"
#include <map>
#include <string>

using namespace std;


const double PI = 3.1415926;
int index_initial = 100;
bool is_block = false;
std::map<int,string> mapErr= {
    {2,"ERR_FUCTION_CALL_ERROR"},
    {-1,"ERR_INVALID_HANDLER"},
    {-2,"ERR_INVALID_PARAMETER"},
    {-3,"ERR_COMMUNICATION_ERR"},
    {-4,"ERR_KINE_INVERSE_ERR"},
    {-5,"ERR_EMERGENCY_PRESSED"},
    {-6,"ERR_NOT_POWERED"},
    {-7,"ERR_NOT_ENABLED"},
    {-8,"ERR_DISABLE_SERVOMODE"},
    {-9,"ERR_NOT_OFF_ENABLE"},
    {-10,"ERR_PROGRAM_IS_RUNNING"},
    {-11,"ERR_CANNOT_OPEN_FILE"},
    {-12,"ERR_MOTION_ABNORMAL"}
    };



/* global variables */
JAKAZuRobot robot;

bool stop_callback(std_srvs::Empty::Request &req,
        std_srvs::Empty::Response &res)
{
    //robot.disable_robot();
    int sdk_res = robot.motion_abort();
    switch(sdk_res)
    {
        case 0:
            ROS_INFO("stop sucess");
            break;
        default:
            cout<<"error:"<<mapErr[sdk_res]<<endl;
            // ROS_INFO(mapErr[sdk_res]);
            break;
    }
    
    return true;
}

bool set_user_frame_callback(robot_msgs::SetUserFrame::Request &req,
        robot_msgs::SetUserFrame::Response &res)
{
    CartesianPose cart;
    cart.tran.x = req.pose[0]*1000;
    cart.tran.y = req.pose[1]*1000;
    cart.tran.z = req.pose[2]*1000;
    /* cart.rpy.rx = req.pose[3];
    cart.rpy.ry = req.pose[4];
    cart.rpy.rz = req.pose[5]; */

    // ROS_INFO("srv");
    //Angaxis2rpy
    Eigen::Vector3d Angaxis = {req.pose[3], req.pose[4], req.pose[5]};
    RotMatrix Rot = Angaxis2Rot(Angaxis);
    robot.rot_matrix_to_rpy(&Rot, &(cart.rpy));

    int id = req.user_num;

    
    // ROS_INFO("set");
    res.ret = robot.set_user_frame_data(id, &cart,"Base Coord");
    // ROS_INFO("end");
    switch(res.ret)
    {
        case 0:
            res.message = "User frame is set";
            break;
        default:
            res.message = "Failed to set user frame";
            break;
    }
    return true;
}

/* bool req_user_frame_callback(robot_msgs::SetUserFrame::Request &req,
        robot_msgs::SetUserFrame::Response &res)
{


    return true;

} */

bool set_tcp(robot_msgs::SetTcp::Request &req,
        robot_msgs::SetTcp::Response &res)
{
    CartesianPose cart;
    cart.tran.x = req.pose[0]*1000;
    cart.tran.y = req.pose[1]*1000;
    cart.tran.z = req.pose[2]*1000;

    //Angaxis2rpy
    Eigen::Vector3d Angaxis = {req.pose[3], req.pose[4], req.pose[5]};
    RotMatrix Rot = Angaxis2Rot(Angaxis);
    
    if(robot.rot_matrix_to_rpy(&Rot, &(cart.rpy))) ROS_INFO("rot_matrix_to_rpy failed!!!!!!");
    
    // int id = req.tool_num;
    int id = 1;
    cout<<"pose_tcp"<<endl;
    cout <<cart.tran.x<<endl;
    cout <<cart.tran.y<<endl;
    cout <<cart.tran.z<<endl;
    cout <<cart.rpy.rx<<endl;
    cout <<cart.rpy.ry<<endl;
    cout <<cart.rpy.rz<<endl;

    int sdk_res_data = robot.set_tool_data(id, &cart, "Tool Coord"); // 接受sdk返回值
    int sdk_res_id = robot.set_tool_id(id);//设置id

    // 转换为需求字典

    if (sdk_res_data==0)
    {
        res.ret = 1;
        res.message = "Tcp is set";
    }
    else{
        res.message = mapErr[sdk_res_data];
    }

    if (sdk_res_id==0)
    {
        res.ret = 1;
        res.message = "Tcp is set";
    }
    else{
        res.message = mapErr[sdk_res_id];
    }

    return true;
}


bool enable_teach_drag_callback(std_srvs::SetBool::Request &req,
        std_srvs::SetBool::Response &res)
{
    bool enable_teach_drag = req.data;
    res.success = !robot.drag_mode_enable(enable_teach_drag);
    if(res.success)
    {
        res.message = "OK";
    }
    else
    {
        res.message = "Fail";
    }
    return true;
}

bool enable_servo_callback(std_srvs::SetBool::Request &req,
        std_srvs::SetBool::Response &res)
{
    bool enable_servo = req.data;
    res.success = !robot.servo_move_enable(enable_servo);
    if(res.success)
    {
        res.message = "OK";
    }
    else
    {
        res.message = "Fail";
    }
    return true;
}

bool set_payload_callback(robot_msgs::SetLoad::Request &req,
        robot_msgs::SetLoad::Response &res)
{
    PayLoad payload;
    int tool_id = req.tool_num;

    payload.centroid.x = req.xc*1000;
    payload.centroid.y = req.yc*1000;
    payload.centroid.z = req.zc*1000;
    payload.mass = req.mass;

    robot.set_tool_id(tool_id);
    int sdk_res = robot.set_payload(&payload);

    switch(sdk_res)
    {
        case 0:
            res.message = "Payload is set";
            res.ret = 1;
            break;
        default:
            res.ret = sdk_res;
            res.message = mapErr[sdk_res];
            break;
    }

    return true;
}

bool clear_err_callback(robot_msgs::ClearErr::Request &req,
        robot_msgs::ClearErr::Response &res)
{
    RobotState state;
    RobotStatus status;

    robot.get_robot_state(&state);
    robot.get_robot_status(&status);
    

    // estoped
    // estoped == 0 enable
    ROS_INFO("status.enabled:%d", status.enabled);

    if(!state.estoped){
        res.ret = robot.collision_recover();
        switch(res.ret)
        {
            case 0:
                res.message = "Collision error is clear\n";
                break;
            default:
                // res.message = "Failed to clear error\n";
                res.message = mapErr[res.ret]+"\n";
                break;
        }

        //Robot re-enable
        if(!status.enabled)
        {
            if(robot.power_on())
                ROS_INFO("power_on error!!");
            res.ret = robot.enable_robot();
            switch(res.ret)
            {
                case 0:
                    res.message = "Robot is recovered and enabled";
                    break;
                default:
                    // res.message = "Failed to enable robot";
                    res.message = mapErr[res.ret] + "\n";
                    break;
            }
        }
        //Robot collision recover
        
    }
    // res.ret = robot.enable_robot();
    // estoped == 1 un_enable
    return true;
}

bool set_collision_callback(robot_msgs::SetCollision::Request &req,
        robot_msgs::SetCollision::Response &res)
{
    int collision_level = 0;
    std::string msg = "Collision level ";

    if(req.is_enable == 0)
    {
        collision_level = 0;
    }
    else
    {
        if(req.value <= 25)
        {
            collision_level = 1;
        }
        else if(req.value <= 50)
        {
            collision_level = 2;
        }
        else if(req.value <= 75)
        {
            collision_level = 3;
        }
        else if(req.value <=100)
        {
            collision_level = 4;
        }
        else
        {
            collision_level = 5;
        }
    }
    res.ret = robot.set_collision_level(collision_level);

    switch(res.ret)
    {
        case 0:
            
            msg = msg + to_string(collision_level) + " is set";
            res.message = msg;
            break;
        default:
            res.message = mapErr[res.ret] + "\n";
            break;
    }

    return true;
}

bool motion_ctrl_callback(robot_msgs::SetAxis::Request &req,
        robot_msgs::SetAxis::Response &res)
{
    res.ret = 1;
    res.message = "OK";

    return true;
}

bool movel_callback(robot_msgs::Move::Request &req,
        robot_msgs::Move::Response &res)
{   
    // ROS_INFO("movel_called!!!");
    CartesianPose cart;
    cart.tran.x = req.pose[0]*1000;
    cart.tran.y = req.pose[1]*1000;
    cart.tran.z = req.pose[2]*1000;
    /* cart.rpy.rx = req.pose[3];
    cart.rpy.ry = req.pose[4];
    cart.rpy.rz = req.pose[5];
    RotMatrix Rot;
    robot.rpy_to_rot_matrix(&(cart.rpy), &Rot);
    Eigen::Vector3d Angaxis = Rot2Angaxis(Rot);
    cart.rpy.rx = Angaxis[0];
    cart.rpy.ry = Angaxis[1];
    cart.rpy.rz = Angaxis[2]; */

    //Angaxis2rpy
    Eigen::Vector3d Angaxis = {req.pose[3], req.pose[4], req.pose[5]};
    RotMatrix Rot = Angaxis2Rot(Angaxis);
    robot.rot_matrix_to_rpy(&Rot, &(cart.rpy));
    
    double speed = (double)req.mvvelo*1000;
    double accel = (double)req.mvacc*1000;

    OptionalCond* p = nullptr;

    int sdk_res = robot.linear_move(&cart,MoveMode::ABS,is_block,speed,accel,0.3,p);
    // cout<<"sdk_res:"<<sdk_res<<endl;
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

bool movej_callback(robot_msgs::Move::Request &req,
        robot_msgs::Move::Response &res)
{   
    JointValue joint_pose;
    joint_pose.jVal[0] = (req.pose[0]);
    joint_pose.jVal[1] = (req.pose[1]);
    joint_pose.jVal[2] = (req.pose[2]);
    joint_pose.jVal[3] = (req.pose[3]);
    joint_pose.jVal[4] = (req.pose[4]);
    joint_pose.jVal[5] = (req.pose[5]);

    double speed = (double)req.mvvelo;
    double accel = (double)req.mvacc;

    OptionalCond* p = nullptr;

    int sdk_res = robot.joint_move(&joint_pose,MoveMode::ABS,is_block,speed,accel,0.2,p);

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


/* move_jog */
/* bool move_jog_callback(robot_msgs::Move::Request &req,
        robot_msgs::Move::Response &res)
{

    //1 赋初始值
    double move_velocity = req.mvvelo;
    CoordType coord_type = COORD_BASE;
    double pos_cmd = 0.0;
    
    //2 选择index   mapping the index and velocity
    //req.index 如果是关节空间   就是  0+,0-,1+,1-,2+,2-,3+,3-,4+,4-,5+,5-
    //req.index 如果是笛卡尔空间 就是x+,x-,y+,y-,z+,z-,rx+,rx-,ry+,ry-,rz+,rz-
    float index_f = static_cast<float>(req.index) / 2 + 0.1;
    int index = static_cast<int>(index_f);
    
    //3 选择坐标
    switch (req.coord_mode)
    {
        case 0:
            coord_type = COORD_JOINT; //关节坐标 关节空间 
            pos_cmd = PI / 180.0;       //某个关节运动(PI/180.0)弧度 (可修改)
            break;
        case 1:
            coord_type = COORD_BASE;   //基坐标 笛卡尔空间
            if (index >= 3)
            {                         //沿rx,ry,rz，旋转(PI/180.0)弧度 (可修改)
                pos_cmd = PI / 180.0;
            }
			else
			{
				pos_cmd = 10;         //沿x或y或z轴方向，运行10mm  (可修改)   
			}

            break;
        default:
		    coord_type = COORD_TOOL;   //TCP末端坐标 笛卡尔空间
			if (index >= 3)
            {                         //沿rx,ry,rz，旋转(PI/180.0)弧度(可修改)
                pos_cmd = PI / 180.0;
            }
			else
			{
				pos_cmd = 10;         //沿x或y或z轴方向，运行10mm  (可修改)   
			} 
    }
    // 4 确定速度方向 //判断速度是正方向还是负方向
    if(req.index&1)
    {
        move_velocity = -move_velocity;
    }
    //5 进行jog运动
    // cout << index << ";" << coord_type << ";" << move_velocity << ";"<< pos_cmd << endl;
    int jog_stop_res = robot.jog_stop(-1);
    if (jog_stop_res == 0)
    {
        sleep(0.2);
        int sdk_res = robot.jog(index, INCR, coord_type, move_velocity, pos_cmd);
        switch(sdk_res)
        {
            case 0:
                res.ret = 1;
                res.message = "Position is reached";
                break;
            default:
                res.ret = sdk_res;
                res.message = mapErr[sdk_res]+"\n";
                break;
        }
    }
    else
    {
        res.ret = jog_stop_res;
        res.message = mapErr[jog_stop_res]+"\n";
    }
    return true;

} */

bool move_jog_callback(robot_msgs::Move::Request &req,
        robot_msgs::Move::Response &res)
{

    //1 赋初始值
    double move_velocity = 0;
    CoordType coord_type = COORD_JOINT;
    
    //2 选择index   mapping the index and velocity
    //req.index 如果是关节空间   就是  0+,0-,1+,1-,2+,2-,3+,3-,4+,4-,5+,5-
    //req.index 如果是笛卡尔空间 就是x+,x-,y+,y-,z+,z-,rx+,rx-,ry+,ry-,rz+,rz-
    float index_f = static_cast<float>(req.index) / 2 + 0.1;
    int index = static_cast<int>(index_f);
    
    //3 选择坐标
    switch (req.coord_mode)
    {
        case 0:
            coord_type = COORD_JOINT; //关节坐标 关节空间 
            move_velocity = PI / 180.0 * 2;       //关节运动速度 (可修改)
            break;
        case 1:
            coord_type = COORD_BASE;   //基坐标 笛卡尔空间
            if (index >= 3)
            {                         
                move_velocity = PI / 180.0 * 2;     //rx,ry,rz，旋转运动速度 (可修改)
            }
			else
			{
				move_velocity = 2;         //沿x或y或z轴方向运动速度  (可修改)   
			}

            break;
        default:
		    coord_type = COORD_TOOL;   //TCP末端坐标 笛卡尔空间
			if (index >= 3)
            {                         
                move_velocity = PI / 180.0 * 2;     //rx,ry,rz，旋转运动速度 (可修改)
            }
			else
			{
				move_velocity = 2;         //沿x或y或z轴方向运动速度  (可修改)    
			} 
    }
    // 4 确定速度方向 //判断速度是正方向还是负方向
    if(req.index&1)
    {
        move_velocity = -move_velocity;
    }
    //5 进行jog运动
    // cout << index << ";" << coord_type << ";" << move_velocity << ";"<< pos_cmd << endl;
    if (index_initial != req.index)
    {   
        int jog_stop_res = robot.jog_stop(-1);
        if (jog_stop_res == 0)
        {
            sleep(0.2);
            int sdk_res = robot.jog(index, CONTINUE, coord_type, move_velocity, 0);
            switch(sdk_res)
            {
                case 0:
                    res.ret = 1;
                    res.message = "Position is reached";
                    break;
                default:
                    res.ret = sdk_res;
                    res.message = mapErr[sdk_res]+"\n";
                    break;
            }
        }
        else
        {
            res.ret = jog_stop_res;
            res.message = mapErr[jog_stop_res]+"\n";
        }
        index_initial = req.index;
    }
    else
    {
        res.ret = 1;
        res.message = "Moving on";
        ROS_INFO("Moving on");
    }
    return true;
}


/* get robot state */
void get_robot_state_callback(ros::Publisher robot_state_pub)
{
    robot_msgs::RobotMsg robot_state;
    RobotState state;
    robot.get_robot_state(&state);

}

/* enable or disable robot */
void robot_state_control_callback()
{

    bool enable_robot, disable_robot; // enable or disable robot 
    ros::param::get("/enable_robot", enable_robot);
    ros::param::get("/disable_robot", disable_robot);

    if(enable_robot)
    {
        robot.power_on();
        robot.enable_robot();
        ros::param::set("/enable_robot", false);
    }

    if(disable_robot)
    {
        robot.disable_robot();
        //                        robot.power_off();
        ros::param::set("/disable_robot", false);
    }


}

/* publish realtime joint angle */
/* void joint_states_callback(const ros::TimerEvent&, ros::Publisher joint_states_pub)
{
    //        clock_t t1 = clock();

    sensor_msgs::JointState joint_states;
    JointValue joint_pose;

    //        joint_states.position.clear(); // clear position vector
    robot.get_joint_position(&joint_pose); // get current joint pose
    for(int i = 0; i < 6; i++)
    {
        joint_states.position.push_back(joint_pose.jVal[i]); // write data into standard ros msg
    }

    joint_states_pub.publish(joint_states); // publish data

    //        cout << (clock() - t1) * 1.0 / CLOCKS_PER_SEC * 1000 << endl;


} */

//1.4 servo line publish
/* void Publish_Servo_Line(ros::Publisher& servo_line)
{
    jaka_ros_driver::ServoL set_pose;

    servo_line.publish(set_pose);

} */
//1.4 servo line publish
void servo_line_callback(ros::Publisher servo_line)
{

    robot_msgs::ServoL set_pose;
    // float pos[6] = {set_pose[0],set_pose[1],set_pose[2],set_pose[3],set_pose[4],set_pose[5]};
    // robot.servo_p(&set_pose,ABS);
    CartesianPose pose;
    pose.tran.x = set_pose.pose[0];
    pose.tran.y = set_pose.pose[1];
    pose.tran.z = set_pose.pose[2];

    pose.rpy.rx = set_pose.pose[0];
    pose.rpy.ry = set_pose.pose[1];
    pose.rpy.rz = set_pose.pose[2];
    // robot.servo_move_enable(true);

    // robot.servo_p(&pose,ABS);
    // robot.servo_move_enable(false);

    servo_line.publish(set_pose);

}

//2.1 robot tcp publish
void tool_point_callback(ros::Publisher tcp_pub)
{
    geometry_msgs::TwistStamped tool_point;
    CartesianPose tool_pose;
    robot.get_tcp_position(&tool_pose);

    // tool_pose.tran.x = 0;
    // tool_pose.tran.y = 0;
    // tool_pose.tran.z = 0;

    // tool_pose.rpy.rx = 0;
    // tool_pose.rpy.ry = 0;
    // tool_pose.rpy.rz = 0;

    //header
    
    //twist

    //tran
    
    tool_point.twist.linear.x = tool_pose.tran.x/1000;
    tool_point.twist.linear.y = tool_pose.tran.y/1000;
    tool_point.twist.linear.z = tool_pose.tran.z/1000;
    
    //rpy2Angaxis
    RotMatrix Rot;
    robot.rpy_to_rot_matrix(&(tool_pose.rpy), &Rot);
    Eigen::Vector3d Angaxis = Rot2Angaxis(Rot);
    tool_point.twist.angular.x = Angaxis[0];
    tool_point.twist.angular.y = Angaxis[1];
    tool_point.twist.angular.z = Angaxis[2];
    
    tcp_pub.publish(tool_point);
}

//2.2 robot joint publish
void joint_states_callback(ros::Publisher joint_states_pub)
{
    //        clock_t t1 = clock();

    sensor_msgs::JointState joint_states;
    JointValue joint_pose;

    //        joint_states.position.clear(); // clear position vector
    robot.get_joint_position(&joint_pose); // get current joint pose
  //
    // joint_pose.jVal[0] = 0;
    // joint_pose.jVal[1] = 1;
    // joint_pose.jVal[2] = 2;
    // joint_pose.jVal[3] = 3;
    // joint_pose.jVal[4] = 4;
    // joint_pose.jVal[5] = 5;
//
    for(int i = 0; i < 6; i++)
    {
        joint_states.position.push_back(joint_pose.jVal[i]); // write data into standard ros msg
        int j = i+1;
        joint_states.name.push_back("joint_"+std::to_string(j));
        joint_states.header.stamp = ros::Time::now();
    }

    joint_states_pub.publish(joint_states); // publish data

    //        cout << (clock() - t1) * 1.0 / CLOCKS_PER_SEC * 1000 << endl;


}

//2.3 robot state publish
void robot_state_callback(ros::Publisher robot_state_pub)
{
    robot_msgs::RobotMsg robot_state;

    RobotState robotstate;
    RobotStatus robotstatus;
    ProgramState programstate;
    BOOL in_pos = true;
    BOOL in_col = false;
 

    if(robot.is_in_pos(&in_pos))   ROS_INFO("Failed to get robot pos!");
    if(robot.get_program_state(&programstate)) ROS_INFO("Failed to get program state!");
    if(robot.get_robot_status(&robotstatus)) ROS_INFO("Failed to get robot status!");
    if(robot.get_robot_state(&robotstate)) ROS_INFO("Failed to get robot state!");
    if(robot.is_in_collision(&in_col)) ROS_INFO("Failed to get robot collision state!");
    
    if(robotstate.estoped)
    {
        robot_state.state = 2;
        //ROS_INFO("Robot program is running!");
    }
    else if(robotstatus.errcode != 0)
    {
        robot_state.state = 4;
        //ROS_INFO("Robot is in error!")
    }
    else if(in_pos && programstate == PROGRAM_IDLE && robotstatus.drag_status == 0)
    {
        robot_state.state = 0;
        //ROS_INFO("Robot is in pos!");
    }
    else if(programstate == PROGRAM_PAUSED)
    {
        robot_state.state = 1;
        //ROS_INFO("Robot program is paused!");
    }
    else if(!in_pos || programstate == PROGRAM_RUNNING || robotstatus.drag_status == 1)
    {
        robot_state.state = 3;
        //ROS_INFO("Robot program is running!");
    }
    
    /*
    if(in_pos && programstate == PROGRAM_IDLE)
    {
        robot_state.state = 0;
        //ROS_INFO("Robot is in pos!");
    }
    else if(programstate == PROGRAM_PAUSED)
    {
        robot_state.state = 1;
        //ROS_INFO("Robot program is paused!");
    }
    else if(robotstate.estoped)
    {
        robot_state.state = 2;
        //ROS_INFO("Robot is emergency stopped!");
    }
    else if(!in_pos || programstate == PROGRAM_RUNNING)
    {
        robot_state.state = 3;
        //ROS_INFO("Robot program is running!");
    }
    else if(robotstatus.errcode != 0)
    {
        robot_state.state = 4;
        //ROS_INFO("Robot is in error!");
    }
    */
   
    robot_state.mode = 2;
    //ROS_INFO("Robot is in distance mode!");

    
    if(robotstate.poweredOn)
    {
        robot_state.motor_sync = 1;
        //ROS_INFO("Robot is in synchronization!");
    }
    else
    {
        robot_state.motor_sync = 0;
        //ROS_INFO("Robot is in unsynchronization!");
    }

    if(robotstatus.enabled)
    {
        //客户文档要求servo_enable 代表机器人是否使能，sdk内的servo_enable表示透传模式下是否使能
        robot_state.servo_enable = 1;
        //ROS_INFO("Servo mode is enable!");
    }
    else
    {
        robot_state.servo_enable = 0;
        //ROS_INFO("Servo mode is disable!");
    }

    if(in_col)
    {
        robot_state.collision_state = 1;
        //ROS_INFO("Robot is in collision!");
    }
    else
    {
        robot_state.collision_state = 0;
        //ROS_INFO("Robot is not in collision!");
    }
    // robot_state.state = 0;
    // robot_state.mode = 2;
    // robot_state.servo_enable = 1;
    // robot_state.motor_sync = 1;
    // robot_state.collision_state = 0;

    robot_state_pub.publish(robot_state);

}

// 多线程监控网络状态
void* get_conn_scoket_state(void* args){
    int get_count = 0;
    RobotStatus robot_status;
    int is_socket_connect;
    while (ros::ok())
    {
        /* code */
        int ret = robot.get_robot_status(&robot_status);

        // cout<<"ret:"<<ret<<endl;
        is_socket_connect = robot_status.is_socket_connect;
        // cout<<"postition:"<<robot_status.joint_position[0]<<endl;
        // cout<<"is_socket_connect:"<<is_socket_connect<<endl;
        if(!is_socket_connect){
            ROS_ERROR("connect error!!!");
        }
        // cout<<"get_count:"<<get_count++<<endl;
        
        sleep(1);
    }
    
}




int main(int argc, char **argv)
{
    RobotStatus ret_status;
    robot.set_status_data_update_time_interval(40);
    ros::init(argc, argv, "connect_robot");
    ros::NodeHandle n;

    // init params
    ros::param::set("/enable_robot", false);
    ros::param::set("/disable_robot", false);
    //ros::param::set("robot_ip","10.5.5.100");
    std::string ip;
    ros::param::get("robot_ip",ip);
    //ROS_INFO("%s",ip.c_str());
    /* services and topics */

    // 1.1 service move line -
    ros::ServiceServer service_movel = n.advertiseService("/robot_driver/move_line", movel_callback);

    // 1.2 service move joint -
    ros::ServiceServer service_movej = n.advertiseService("/robot_driver/move_joint", movej_callback);

    // 1.3 service jog move
    ros::ServiceServer move_jog = n.advertiseService("/robot_driver/move_jog", move_jog_callback);

    // 1.4 service servo line -
    ros::Publisher servo_line_pub = n.advertise<robot_msgs::ServoL>("/robot_driver/servo_line", 100);

    // 1.5 service stop move -
    ros::ServiceServer service_stop = n.advertiseService("/robot_driver/stop_move",stop_callback);

    // 2.1 robot tcp publisher -
    ros::Publisher tool_point_pub = n.advertise<geometry_msgs::TwistStamped>("/robot_driver/tool_point", 10);

    // 2.2 read joint state from sdk and publish it to /robot_driver/robot_states -
    ros::Publisher joint_states_pub = n.advertise<sensor_msgs::JointState>("/robot_driver/joint_states", 10);

    // 2.3 robot state publisher -
    ros::Publisher robot_state_pub = n.advertise<robot_msgs::RobotMsg>("/robot_driver/robot_states", 10);

    // 3.1 service set user frame -
    ros::ServiceServer service_set_user_frame = n.advertiseService("/robot_driver/set_user_frame", set_user_frame_callback);
    // ros::ServiceServer service_req_user_frame = n.advertiseService("/robot_driver/set_user_frame", req_user_frame_callback);

    // 3.2 service set tcp -
    ros::ServiceServer service_set_tcp = n.advertiseService("/robot_driver/set_tcp", set_tcp);

    // 3.3 service enable teach drag -
    ros::ServiceServer service_enable_teach_drag = n.advertiseService("/robot_driver/teach_drag", enable_teach_drag_callback);

    // 3.4 service enable servo -
    ros::ServiceServer service_enable_servo = n.advertiseService("/robot_driver/servo_ctr", enable_servo_callback);

    // 3.5 service set payload
    ros::ServiceServer service_set_payload = n.advertiseService("/robot_driver/set_payload", set_payload_callback);

    // 3.6 service clear error
    ros::ServiceServer service_clear_err = n.advertiseService("/robot_driver/clear_err", clear_err_callback);

    // 3.7 service set collision level
    ros::ServiceServer service_set_collision = n.advertiseService("/robot_driver/set_collision", set_collision_callback);

    // 3.8
    ros::ServiceServer service_motion_ctrl = n.advertiseService("/robot_driver/motion_ctrl", motion_ctrl_callback);

    // ros::Rate loop_rate(10);
    ROS_INFO("Try to connect robot");

    // connect robot
    std::cout<<"-----------"<<endl;
    // std::cout<<robot.login_in("10.5.5.100")<<std::endl;
    robot.login_in(ip.c_str());
    //robot.login_in("10.5.5.100");
    //std::cout<<robot.login_in("192.168.51.15")<<std::endl;
    robot.set_network_exception_handle(110, MOT_ABORT); 
    sleep(1);
    robot.power_on();
    sleep(1);
    robot.enable_robot();
    robot.get_robot_status(&ret_status);
    if (ret_status.enabled == false)
    { 
        ROS_INFO("restart......");
        robot.power_off();
        sleep(1);
        robot.power_on();
        sleep(1);
        robot.enable_robot(); 
    }

    ROS_INFO("Robot:%s enable",ip.c_str());
    ros::Duration(1).sleep();
    ROS_INFO("Robot:%s ready!",ip.c_str());
    // 监控状态
    pthread_t conn_state_thread;
    int ret = pthread_create(&conn_state_thread,NULL,get_conn_scoket_state,NULL);

    if(ret!=0){
        ROS_ERROR("thread creat error!!!!!!!!!!");
    }

    // robot control timer
    // ros::Timer robot_state_control_timer = n.createTimer(ros::Duration(1), robot_state_control_callback);

    // // 1.4
    // ros::Timer servo_line_timer = n.createTimer(ros::Duration(0.08), boost::bind(servo_line_callback, _1, servo_line_pub));

    // // 2.1
    // ros::Timer tool_point_timer = n.createTimer(ros::Duration(0.08), boost::bind(tool_point_callback, _1, tool_point_pub));

    // // 2.2
    // ros::Timer joint_states_timer = n.createTimer(ros::Duration(0.08), boost::bind(joint_states_callback, _1, joint_states_pub));

    // // 2.3
    // ros::Timer robot_states_timer = n.createTimer(ros::Duration(0.08), boost::bind(robot_state_callback, _1, robot_state_pub));

    // ros::spin();
  // ros::Timer robot_state_control_timer = n.createTimer(ros::Duration(1), robot_state_control_callback);

    // // 1.4
    // ros::Timer servo_line_timer = n.createTimer(ros::Duration(0.08), boost::bind(servo_line_callback, _1, servo_line_pub));

    // // 2.1
    // ros::Timer tool_point_timer = n.createTimer(ros::Duration(0.08), boost::bind(tool_point_callback, _1, tool_point_pub));

    // // 2.2
    // ros::Timer joint_states_timer = n.createTimer(ros::Duration(0.08), boost::bind(joint_states_callback, _1, joint_states_pub));

    // // 2.3
    // ros::Timer robot_states_timer = n.createTimer(ros::Duration(0.08), boost::bind(robot_state_callback, _1, robot_state_pub));
    ros::Rate rate(5);
    int count = 0;
    // ros::spin();
    while (ros::ok())
    {
        
        // 1.4
        servo_line_callback(servo_line_pub);
        // 2.1
        tool_point_callback(tool_point_pub);
        //2.2
        joint_states_callback(joint_states_pub);
        // 2.3
        robot_state_callback(robot_state_pub);
        
        ros::spinOnce();
        count++;
        // ROS_INFO("count:");
        // cout<<count<<endl;
        rate.sleep();
        // ros::Rate::sleep();
    }
    return 0;
}

