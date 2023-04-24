#include "ros/ros.h"

#include "std_msgs/String.h"

#include "Eigen/Dense"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/StdVector"


#include "std_srvs/Empty.h"
#include "std_srvs/SetBool.h"
#include "robot_msgs/RobotMsg.h"
#include "robot_msgs/SetUserFrame.h"
#include "robot_msgs/SetTcp.h"

#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"


#include <sstream>

#include <vector>

#include "libs/robot.h"
#include "libs/conversion.h"
#include "time.h"

using namespace std;

/* global variables */
JAKAZuRobot robot;

int main(int argc, char **argv)
{
    // connect robot
    std::cout<<robot.login_in("192.168.3.101")<<std::endl;
    std::cout<<" robot login in"<<std::endl;
    //std::cout<<robot.login_in("192.168.51.15")<<std::endl;
    std::cout<<robot.power_on();
    std::cout<<" robot power on"<<std::endl;
    std::cout<<robot.enable_robot();
    std::cout<<" robot enable"<<std::endl;

    sleep(3);
    std::cout<<" robot disable111"<<std::endl;
    std::cout<<robot.disable_robot();
    std::cout<<" robot disable"<<std::endl;
    std::cout<<robot.power_off();
    std::cout<<" robot power off"<<std::endl;
    std::cout<<robot.login_out();
    std::cout<<" robot login out"<<std::endl;

    return 0;
}