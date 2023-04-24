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
    ros::init(argc, argv, "robot_client"); 
    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<robot_msgs::SetTcp>("/robot_driver/set_tcp");
    ros::service::waitForService("/robot_driver/set_tcp");
    //ros::ServiceClient client = n.serviceClient<std_srvs::SetBool>("/robot_driver/teach_drag");
    //ros::service::waitForService("/robot_driver/teach_drag");

    robot_msgs::SetTcp A;
    float array[6] = {0.03,0.04,0,0,0,0};
    std::vector<float> array1;
    for(int i=0;i<=6;++i){array1.push_back(array[i]);}
    A.request.pose = array1;
    A.request.tool_num = 1;

    bool flag = client.call(A);
    if(flag)
    {
        ROS_INFO("ok");
    }
    else
    {
        ROS_ERROR("no");
    }

    //ros::spin();


    return 0;
}