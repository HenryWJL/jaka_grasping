#include "ros/ros.h"
#include "std_msgs/String.h"

#include "Eigen/Dense"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/StdVector"

#include "robot_msgs/Move.h"
#include "std_srvs/Empty.h"
#include "std_srvs/SetBool.h"
#include "robot_msgs/RobotMsg.h"
#include "robot_msgs/SetUserFrame.h"
#include "robot_msgs/SetTcp.h"
#include "robot_msgs/SetLoad.h"
#include "robot_msgs/ServoL.h"
#include "robot_msgs/ClearErr.h"
#include "robot_msgs/SetCollision.h"

#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"

#include <pthread.h>
#include <unistd.h>
#include <sstream>

#include <vector>

#include "libs/robot.h"
#include "libs/conversion.h"
#include "time.h"
#include <map>
#include <string>
using namespace std;

void robot_state_callF(const robot_msgs::RobotMsg::ConstPtr& msg_p)
{
    ROS_INFO("%d", msg_p->state);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_sub"); 
    ros::NodeHandle n;

    ros::Subscriber robot_state_sub = n.subscribe("/robot_driver/robot_states", 10, robot_state_callF);

    ros::spin();

    return 0;
}