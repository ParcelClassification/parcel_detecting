#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "geometry_msgs/TransformStamped.h"
#include "std_msgs/Float32.h"
#include "parcel_msgs/parcel_poses.h"

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"

#include <set>
#include <map>
#include <cmath>
#include <fstream>

using namespace cv;
using namespace std;

class ParcelYolo
{
public:
  ParcelYolo(string topicname);
  ~ParcelYolo();
  Mat opencvImage;
  bool find_parcel();
private:
  ros::ServiceClient yolo_client;
  sensor_msgs::Image Image;
  parcel_msgs::parcel_poses parcel_list;
  ros::NodeHandle nh_;
  cv_bridge::CvImagePtr cv_ptr;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub;
  string topic_name;
  void imageCb(const sensor_msgs::ImageConstPtr& msg);
};