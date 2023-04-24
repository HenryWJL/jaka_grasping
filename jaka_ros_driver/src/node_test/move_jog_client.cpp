#include "ros/ros.h"

#include "robot_msgs/Move.h"

#include "string"

#include "libs/robot.h"

using namespace std;

const double PI = 3.1415926;

int main(int argc, char **argv)
{
    ros::init(argc,argv,"move_jog_client");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<robot_msgs::Move>("/robot_driver/move_jog");

    robot_msgs::Move srv;
    
    srv.request.mvvelo = -15;
    srv.request.coord_mode = 2;
    //uncertain
    srv.request.index = 10;


    if(client.call(srv))
    {
        ROS_INFO("Response from server message: %s",srv.response.message.c_str());
        ROS_INFO("Response from server ret: %d",srv.response.ret);
    }else{
        ROS_ERROR("failed to call /robot_driver/move_jog");
        return 1;
    }

    return 0;

}
