#include "ros/ros.h"
#include "parcel_yolo.cpp"
#include <string>

using namespace std;
using namespace cv;

ParcelYolo *finder;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "parcel_main");
  ros::NodeHandle nh;
  string topic = "image topic name";
  finder = new ParcelYolo(topic);
  ROS_INFO("READY PARCEL MAIN");
  //service finder->find_parcel();
  ros::spin();
  delete finder;
  return 0;
}