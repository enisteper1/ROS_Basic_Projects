#include "ros/ros.h"
#include "ros_projects/RectangleArea.h"

bool calculate(ros_projects::RectangleArea::Request  &req,
         ros_projects::RectangleArea::Response &res)
{
  res.area = req.width * req.height;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.width, (long int)req.height);
  ROS_INFO("sending back response: [%ld]", (long int)res.area);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rectangle_area_server");
  ros::NodeHandle server_node;

  ros::ServiceServer service = server_node.advertiseService("rectangle_area", calculate);
  ROS_INFO("Ready to calculate the area");
  ros::spin();

  return 0;
}
