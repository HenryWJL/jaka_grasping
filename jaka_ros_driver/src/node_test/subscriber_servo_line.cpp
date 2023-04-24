#include "ros/ros.h"

#include "std_msgs/String.h"

#include "Eigen/Dense"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/StdVector"

#include "robot_msgs/ServoL.h"

#include <sstream>

#include <vector>

#include "libs/robot.h"
#include "libs/conversion.h"

const double PI = 3.1415926;

using namespace std;

JAKAZuRobot robot;

void servo_line(const robot_msgs::ServoL& set_pose)
{
        /* if(robot.servo_move_enable(true)){
                ROS_INFO("Servo Enable Falied!");
        } */
        

        CartesianPose Set_Cart;
        
        Set_Cart.tran.x = set_pose.pose[0]*1000;
        Set_Cart.tran.y = set_pose.pose[1]*1000;
        Set_Cart.tran.z = set_pose.pose[2]*1000;
        /* Set_Cart.rpy.rx = set_pose.pose[3];
        Set_Cart.rpy.ry = set_pose.pose[4];
        Set_Cart.rpy.rz = set_pose.pose[5]; */

        //Angaxis2rpy
        std::cout<<Set_Cart.tran.x<<std::endl;
        Eigen::Vector3d Angaxis = {set_pose.pose[3], set_pose.pose[4], set_pose.pose[5]};
        RotMatrix Rot = Angaxis2Rot(Angaxis);
        robot.rot_matrix_to_rpy(&Rot, &(Set_Cart.rpy));

        ROS_INFO("%f %f %f %f %f %f",Set_Cart.tran.x,Set_Cart.tran.y,Set_Cart.tran.z,Set_Cart.rpy.rx,Set_Cart.rpy.ry,Set_Cart.rpy.rz); 
        //std::cout<<set_pose.pose[0];
                /* if(!robot.servo_p(&Set_Cart, MoveMode::ABS)){
                ROS_INFO("Servo Move Failed!");
        } */
        

}



int main(int argc, char **argv)
{
        ros::init(argc, argv, "subscriber_servo_line");

        ros::NodeHandle n;
        /* cout<<robot.login_in("192.168.2.167")<<endl;
        robot.power_on();
        robot.enable_robot(); */

        // read joint state from sdk and publish it to /robot_driver/servo_line
        ros::Subscriber servo_line_sub = n.subscribe("/robot_driver/servo_line", 10, servo_line);

        ros::spin();

        return 0;
}