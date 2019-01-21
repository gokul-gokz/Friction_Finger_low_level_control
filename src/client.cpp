#include "dynamixel_motors/dynamixel_node.hpp"
#include "ros/ros.h"


int main(int argc, char *argv[])
{
 ros::init(argc,argv,"Sending_command_positions");
 ros::NodeHandle n;

 ros::service::waitForService("set_operating_mode");
 ros::service::waitForService("cmd_pos");
 ros::service::waitForService("cmd_torque_ind");
 

 ros::ServiceClient client=n.serviceClient<common_msgs_gl::SendDoubleArray>("cmd_pos");
 
 common_msgs_gl::SendDoubleArray srv;
 //srv.request.data ={atoll(argv[1]),atoll(argv[2])};
 double x1=atoll(argv[1]);
 double x2=atof(argv[2]);
 std::cout<<x1<<x2;
 srv.request.data ={x1,x2};
 
if(client.call(srv))
 {
   ROS_INFO("Success");
 }
 else
 {
  ROS_INFO("Failure");
 }

 //ros::init(argc,argv,"HandController");
 ros::NodeHandle n1;
 ros::service::waitForService("set_operating_mode");
 ros::ServiceClient client_operating_mode=n1.serviceClient<common_msgs_gl::SendIntArray>("set_operating_mode");
 common_msgs_gl::SendIntArray srv1;
 int x3=atoll(argv[3]);
 int x4=atoll(argv[4]);
 std::cout<<x3<<x4;
 srv1.request.data ={x3,x4};
 
if(client_operating_mode.call(srv1))
 {
   ROS_INFO("Success");
 }
 else
 {
  ROS_INFO("Failure");
 }

 return 0;
}
