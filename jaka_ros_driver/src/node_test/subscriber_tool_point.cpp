#include "ros/ros.h"

#include "std_msgs/String.h"

#include "Eigen/Dense"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/StdVector"

// #include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"

#include <sstream>

#include <vector>

#include "libs/robot.h"
#include "libs/conversion.h"

const double PI = 3.1415926;

using namespace std;

JAKAZuRobot robot;

void tool_point_callback(const geometry_msgs::TwistStamped& tool_point)
{
    ROS_INFO("I heard: linear[%f, %f, %f] angular[%f, %f, %f]", 
    tool_point.twist.linear.x, tool_point.twist.linear.y, tool_point.twist.linear.z,
    tool_point.twist.angular.x, tool_point.twist.angular.y, tool_point.twist.angular.z);
}



int main(int argc, char **argv)
{
        ros::init(argc, argv, "subscriber_tool_point");

        ros::NodeHandle n;
        /* cout<<robot.login_in("192.168.2.167")<<endl;
        robot.power_on();
        robot.enable_robot(); */

        //
        ros::Subscriber servo_line_sub = n.subscribe("/robot_driver/tool_point", 10, tool_point_callback);

        ros::spin();

        return 0;
}


