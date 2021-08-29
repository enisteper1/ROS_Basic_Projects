#include "ros/ros.h"
#include "ros_projects/RectangleArea.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rectangle_area_client");
  if (argc != 3)
  {
    ROS_INFO("usage: rectangle_area width height");
    return 1;
  }

  ros::NodeHandle client_node;
  ros::ServiceClient client = client_node.serviceClient<ros_projects::RectangleArea>("rectangle_area");
  // Define server
  ros_projects::RectangleArea server;
  //Width param
  server.request.width = atoll(argv[1]);
  //Height param
  server.request.height = atoll(argv[2]);
  
  if (client.call(server))
  {
    ROS_INFO("Area: %ld", (long int)server.response.area);
  }
  else
  {
    ROS_ERROR("Failed to call service rectangle_area");
    return 1;
  }

  return 0;
}
