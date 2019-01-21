#include "dynamixel_motors/dynamixel_node.hpp"
#include "ros/ros.h"

int main(int argc, char **argv)
{	
	ros::init(argc, argv, "talkerscs");
	std::vector<int> n;
	n.push_back(21);
	n.push_back(20);
	DynamixelNode D1("XM",n);
	ros::spin();
	return 0;
}
