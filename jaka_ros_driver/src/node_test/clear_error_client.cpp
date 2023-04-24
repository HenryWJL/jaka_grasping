#include "ros/ros.h"

#include "robot_msgs/ClearErr.h"

#include "string"

#include "libs/robot.h"

using namespace std;

JAKAZuRobot robot;

int main(int argc, char **argv)
{
    ros::init(argc,argv,"clear_error_client");

    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<robot_msgs::ClearErr>("/robot_driver/clear_err");

    robot_msgs::ClearErr srv;
    

    if(client.call(srv))
    {
        ROS_INFO("Response from server message: %s",srv.response.message.c_str());
        ROS_INFO("Response from server ret: %d",srv.response.ret);
    }else{
        ROS_ERROR("failed to call /robot_driver/clear_err");
        return 1;
    }


    return 0;
}
