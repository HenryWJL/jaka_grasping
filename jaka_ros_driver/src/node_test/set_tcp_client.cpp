#include "ros/ros.h"

//#include "std_srvs/SetBool.h"
#include "robot_msgs/SetTcp.h"

#include "string"

#include "libs/robot.h"

using namespace std;

JAKAZuRobot robot;

const double PI = 3.1415926;

int main(int argc, char **argv)
{
    ros::init(argc,argv,"set_tcp_client");

    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<robot_msgs::SetTcp>("/robot_driver/set_tcp");

    robot_msgs::SetTcp srv;

    srv.request.pose.clear();

    srv.request.pose.push_back(0.0);
    srv.request.pose.push_back(0.0);
    srv.request.pose.push_back(0.0);
    srv.request.pose.push_back(0.0);
    srv.request.pose.push_back(0.0);
    srv.request.pose.push_back(0.0);

    srv.request.tool_num = 2;

    if(client.call(srv))
    {
        ROS_INFO("Response from server message: %s",srv.response.message.c_str());
        ROS_INFO("Response from server ret: %d",srv.response.ret);
    }else{
        ROS_ERROR("failed to call /robot_driver/set_tcp");
        return 1;
    }

    //robot.get_tool_data()



    return 0;
}
