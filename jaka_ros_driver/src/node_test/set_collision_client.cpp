#include "ros/ros.h"

#include "robot_msgs/SetCollision.h"

#include "string"

#include "libs/robot.h"

using namespace std;

JAKAZuRobot robot;

int main(int argc, char **argv)
{
    ros::init(argc,argv,"set_collision_client");

    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<robot_msgs::SetCollision>("/robot_driver/set_collision");

    robot_msgs::SetCollision srv;

    srv.request.is_enable = 1;
    srv.request.value = 50;
    

    if(client.call(srv))
    {
        ROS_INFO("Response from server message: %s",srv.response.message.c_str());
        ROS_INFO("Response from server ret: %d",srv.response.ret);
    }else{
        ROS_ERROR("failed to call /robot_driver/set_collision");
        return 1;
    }

    return 0;
}
