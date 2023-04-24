#include "ros/ros.h"

#include "std_srvs/Empty.h"

#include "string"

#include "libs/robot.h"

using namespace std;

const double PI = 3.1415926;

int main(int argc, char **argv)
{
    ros::init(argc,argv,"stop_move_client");

    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<std_srvs::Empty>("/robot_driver/stop_move");

    std_srvs::Empty srv;

    if(client.call(srv))
    {
        ROS_INFO("Jog stop");
    }else{
        ROS_ERROR("failed to call /robot_driver/stop_move");
        return 1;
    }

    return 0;
}
