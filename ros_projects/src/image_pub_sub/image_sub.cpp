#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/highgui.hpp>


void ImageCallback(const sensor_msgs::ImageConstPtr &msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
	 cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception &e)
	{
	 ROS_ERROR("Cv_Bridge Exception %s", e.what());
	 return;
	}

	cv::circle(cv_ptr->image, cv::Point(120, 120),10, CV_RGB(0, 255, 0));

	cv::imshow("Cam", cv_ptr->image);
	cv::waitKey(1);
}


int main(int argc, char** argv)
{
 ros::init(argc, argv, "image_converter");
 ros::NodeHandle sub_node;
 image_transport::ImageTransport im_tp(sub_node);
 image_transport::Subscriber image_sub;

 image_sub = im_tp.subscribe("/camera/image", 1, ImageCallback);
 ros::spin();

 return 0;
}
