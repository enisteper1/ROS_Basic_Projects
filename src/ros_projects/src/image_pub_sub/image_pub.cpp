#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv4/opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
 ros::init(argc, argv, "image_pub");
 ros::NodeHandle node;
 ros::Rate loop_rate(30);
 image_transport::ImageTransport it(node);
 image_transport::Publisher pub = it.advertise("camera/image", 1);
 sensor_msgs::ImagePtr msg;
 Mat frame;
 VideoCapture cap("/home/enis/ROS_Basic_Projects/src/ros_projects/src/resources/tennis-ball-video.mp4");

 if(!cap.isOpened())
 {
  cout<<"Could not reach the camera!"<<endl;
  return -1;
 }

 while(true)
 {
  cap>>frame;
  waitKey(1);
  msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
  if (node.ok())
  {
   pub.publish(msg);
   ros::spinOnce();
   loop_rate.sleep();
  }

 }
 return 0;
}
