#include "ros/ros.h"

#include "std_msgs/String.h"

#include "Eigen/Dense"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/StdVector"


#include "sensor_msgs/JointState.h"
//#include "geometry_msgs/Twist.h"

#include <sstream>

#include <vector>

#include "libs/robot.h"
#include "libs/conversion.h"

const double PI = 3.1415926;

using namespace std;

JAKAZuRobot robot;

void joint_states_callback(const sensor_msgs::JointState& joint_states)
{
    ROS_INFO("I heard: joint1:[%f], joint2:[%f], joint3:[%f], joint4:[%f], joint5:[%f], joint6:[%f]", 
    joint_states.position[0], joint_states.position[1], joint_states.position[2],
    joint_states.position[3], joint_states.position[4], joint_states.position[5]);
}



int main(int argc, char **argv)
{
        ros::init(argc, argv, "subscriber_joint_states");

        ros::NodeHandle n;
        /* cout<<robot.login_in("192.168.2.167")<<endl;
        robot.power_on();
        robot.enable_robot(); */

        //
        ros::Subscriber servo_line_sub = n.subscribe("/robot_driver/joint_states", 10, joint_states_callback);

        ros::spin();

        return 0;
}


