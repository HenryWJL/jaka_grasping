#include "ros/ros.h"

#include "robot_msgs/Move.h"

#include "string"

#include "libs/robot.h"

using namespace std;

const double PI = 3.1415926;

int main(int argc, char **argv)
{
    ros::init(argc,argv,"move_line_client");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<robot_msgs::Move>("/robot_driver/move_line");

    robot_msgs::Move srv;
    srv.request.pose.clear();
    //srv.request.pose[0] = -300.0/1000;
    //srv.request.pose[1] = 98.410/1000;
    //srv.request.pose[2] = 746.832/1000;
    //srv.request.pose[3] = -109.587*PI/180;
    //srv.request.pose[4] = 1.489*PI/180;
    //srv.request.pose[5] = 94.260*PI/180;
    srv.request.mvvelo = 300;
    srv.request.mvacc = 20;
    /* srv.request.pose.push_back(-172.0/1000);
    srv.request.pose.push_back(251.0/1000);
    srv.request.pose.push_back(226.0/1000);
    srv.request.pose.push_back(-90.995*PI/180);
    srv.request.pose.push_back(39.660*PI/180);
    srv.request.pose.push_back(89.877*PI/180); */

    /*srv.request.pose.push_back(-0.427491);
    srv.request.pose.push_back(0.135083);
    srv.request.pose.push_back(0.605524);
    srv.request.pose.push_back(1.928094);
    srv.request.pose.push_back(2.492005);
    srv.request.pose.push_back(-0.398228);*/

    srv.request.pose.push_back(-619.5);
    srv.request.pose.push_back(-21.2);
    srv.request.pose.push_back(552.5);
    srv.request.pose.push_back(-0.68);
    srv.request.pose.push_back(2.88);
    srv.request.pose.push_back(1.103415671);

    //cout<<"ssssssssssssss"<<endl;
// pose: [-719.2,21.096,552.796,-0.68,2.88,1.01]
// has_ref: false
// ref_joint: [0]
// mvvelo: 20.0
// mvacc: 10.0
// mvtime: 0.0
// mvradii: 0.0
// coord_mode: 0
// index: 0

    if(client.call(srv))
    {
        ROS_INFO("Response from server message: %s",srv.response.message.c_str());
        ROS_INFO("Response from server ret: %d",srv.response.ret);


    }else{
        ROS_ERROR("failed to call /robot_driver/move_line");
        return 1;
    }

    //////////////////////////
    
    ros::ServiceClient client_1 = n.serviceClient<robot_msgs::Move>("/robot_driver/move_line");

    robot_msgs::Move srv_1;
    srv_1.request.pose.clear();
    srv_1.request.mvvelo = 300;
    srv_1.request.mvacc = 20;
    srv_1.request.pose.push_back(-719.5);
    srv_1.request.pose.push_back(-21.2);
    srv_1.request.pose.push_back(552.5);
    srv_1.request.pose.push_back(-0.68);
    srv_1.request.pose.push_back(2.88);
    srv_1.request.pose.push_back(1.103415671);
    ROS_INFO("second!!!!");
    if(client_1.call(srv_1))
    {
        ROS_INFO("Response from server message: %s",srv_1.response.message.c_str());
        ROS_INFO("Response from server ret: %d",srv_1.response.ret);


    }else{
        ROS_ERROR("failed to call /robot_driver/move_line");
        return 1;
    }

    return 0;

}
