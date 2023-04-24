#include "ros/ros.h"

#include "std_srvs/SetBool.h"

#include "string"

#include "libs/robot.h"

using namespace std;

const double PI = 3.1415926;

int main(int argc, char **argv)
{
    ros::init(argc,argv,"enable_teach_drag_client");

    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<std_srvs::SetBool>("/robot_driver/teach_drag");

    std_srvs::SetBool srv;

    if(client.call(srv))
    {
        ROS_INFO("success");
    }else{
        ROS_ERROR("failed to call /robot_driver/teach_drag");
        return 1;
    }

    return 0;
}
