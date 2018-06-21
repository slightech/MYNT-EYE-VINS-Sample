#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "cv.h"
#include "highgui.h"
#include <sstream>
#include <cstdlib>
#include <string>

static int i = 1;
static std::string save_image_path = "save_image_path_error";
static std::string pre_model = "support fisheye and pinhole";
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
     cv::namedWindow("img");
     cv::imshow("img", cv_bridge::toCvShare(msg, "8UC1")->image);
     char key =    cv::waitKey(100);
     std::string s = save_image_path + pre_model;
     std::ostringstream oss;
     oss << s << i << ".jpg";
     if(key == 'w')
     {
        cv::imwrite(oss.str(),cv_bridge::toCvShare(msg, "8UC1")->image);
        i++;
     }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "calibration_images");
  ros::NodeHandle nh("~");
  std::string image_topic("topic_error");
  nh.getParam("image_topic", image_topic);
  nh.getParam("save_image_path", save_image_path);
  nh.getParam("pre_model", pre_model);
  ROS_INFO_STREAM("image_topic: " << image_topic);
  ROS_INFO_STREAM("save_image_path: " << save_image_path);
  ROS_INFO_STREAM("pre_model:  " << pre_model);
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe(image_topic, 1, imageCallback);
  ros::spin();
}
