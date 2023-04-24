#include "ros/ros.h"

#include "std_msgs/String.h"

#include "Eigen/Dense"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/StdVector"

#include "robot_msgs/RobotMsg.h"
//#include "sensor_msgs/JointState.h"
//#include "geometry_msgs/Twist.h"

#include <sstream>

#include <vector>

#include "libs/robot.h"
#include "libs/conversion.h"

const double PI = 3.1415926;

using namespace std;

JAKAZuRobot robot;

void robot_state_callback(const robot_msgs::RobotMsg& robot_state)
{
    switch(robot_state.state)
    {
        case 0:
            ROS_INFO("Robot is in pos!");
            break;
        case 1:
            ROS_INFO("Robot program is paused!");
            break;
        case 2:
            ROS_INFO("Robot is emergency stopped!");
            break;
        case 3:
            ROS_INFO("Robot program is running!");
            break;
        default:
            ROS_INFO("Robot is in error!");
            break;
    }

    switch(robot_state.mode)
    {
        case 0:
            ROS_INFO("Robot is in taught mode!");
            break;
        case 1:
            ROS_INFO("Robot is in continuation mode!");
            break;
        default:
            ROS_INFO("Robot is in distance mode!");
            break;
    }

    if(robot_state.motor_sync == 0) 
        ROS_INFO("Robot is out of synchronization!");
    else    
        ROS_INFO("Robot is in synchronization!");

    if(robot_state.servo_enable == 0) 
        ROS_INFO("Servo mode is disable!");
    else    
        ROS_INFO("Servo mode is enable!");
}



int main(int argc, char **argv)
{
        ros::init(argc, argv, "subscriber_robot_states");

        ros::NodeHandle n;
        /* cout<<robot.login_in("192.168.2.167")<<endl;
        robot.power_on();
        robot.enable_robot(); */

        //
        ros::Subscriber servo_line_sub = n.subscribe("/robot_driver/robot_states", 10, robot_state_callback);

        ros::spin();

        return 0;
}


