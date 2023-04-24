#include "ros/ros.h"

//#include "std_srvs/SetBool.h"
#include "robot_msgs/SetUserFrame.h"

#include "string"

#include "libs/robot.h"

using namespace std;

const double PI = 3.1415926;

int main(int argc, char **argv)
{
   
    ros::init(argc,argv,"set_user_frame_client");

    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<robot_msgs::SetUserFrame>("/robot_driver/set_user_frame");


 //std::cout << "set init" << std::endl;
    robot_msgs::SetUserFrame srv;
 //std::cout << "1" << std::endl;
    srv.request.pose.clear();

    srv.request.pose.push_back(-0.102849);
    srv.request.pose.push_back(0.135083);
    srv.request.pose.push_back(0.605524);
    srv.request.pose.push_back(1.928094);
    srv.request.pose.push_back(2.492005);
    srv.request.pose.push_back(-0.398228);
 //std::cout << "2" << std::endl;
    srv.request.user_num = 2;
 //std::cout << "3" << std::endl;

    //std::cout << client.call(srv) << std::endl;
    if(client.call(srv))
    {
        ROS_INFO("Response from server message: %s",srv.response.message.c_str());
        ROS_INFO("Response from server ret: %d",srv.response.ret);
    }else{
        ROS_ERROR("failed to call /robot_driver/set_user_frame");
        return 1;
    }

    return 0;
}
