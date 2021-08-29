#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char** argv)
{
	// Initialize ros
	ros::init(argc, argv, "pub_node");

	//Create node
	ros::NodeHandle pub_node;

	//Define publisher 
	ros::Publisher message_pub = pub_node.advertise<std_msgs::String>("pub_message", 1000);

	// Loop Rate
	ros::Rate rate(1); // 1 Hz

	int counter = 0;
	while(ros::ok())
	{
		// msg will hold  the data
		std_msgs::String message;
		// To define message into std_msgs
		std::stringstream strstream;
		//define message
		strstream<< "Message Published"<< counter;
		
		//Assign the string to message data
		message.data = strstream.str();
		ROS_INFO("Publishing: %s\n", message.data.c_str());

		message_pub.publish(message);

		ros::spinOnce();
		
		rate.sleep();

		counter++;
	}	

	return 0;
}
