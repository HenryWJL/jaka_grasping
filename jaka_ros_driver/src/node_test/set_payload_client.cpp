#include "ros/ros.h"

#include "robot_msgs/SetLoad.h"

#include "string"

#include "libs/robot.h"

using namespace std;

JAKAZuRobot robot;

int main(int argc, char **argv)
{
    ros::init(argc,argv,"set_payload_client");

    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<robot_msgs::SetLoad>("/robot_driver/set_payload");

    robot_msgs::SetLoad srv;

    srv.request.tool_num = 1;
    srv.request.mass = 1;

    srv.request.xc = 0.000;
    srv.request.yc = 0.000;
    srv.request.zc = 0.020;

    

    if(client.call(srv))
    {
        ROS_INFO("Response from server message: %s",srv.response.message.c_str());
        ROS_INFO("Response from server ret: %d",srv.response.ret);
    }else{
        ROS_ERROR("failed to call /robot_driver/set_payload");
        return 1;
    }


    return 0;
}
