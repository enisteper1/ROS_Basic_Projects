#include "ros/ros.h"
#include "std_msgs/String.h"

void callback_message(const std_msgs::String::ConstPtr&);

int main(int argc, char** argv)
{
	// Initializing ros
	ros::init(argc, argv, "sub_node");

	// Defining node to include subscriber
	ros::NodeHandle sub_node;
	
	// Defning subscriber to subscribe pub_message topic
	ros::Subscriber _sub = sub_node.subscribe("pub_message", 1000, callback_message);
	
	// Infinite loop
	ros::spin();

	return 0;
}

void callback_message(const std_msgs::String::ConstPtr& message)
{
	ROS_INFO("Message Recieved: %s\n", message->data.c_str());
}
