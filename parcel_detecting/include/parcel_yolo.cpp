#ifndef PARCEL_YOLO_H
#define PARCEL_YOLO_H

#include "parcel_yolo.h"
#include <stdlib.h>
#include <string>

ParcelYolo::ParcelYolo(string topicname): it_(nh_)
{
  topic_name = topicname;
  image_sub = it_.subscribe(topic_name, 1, &ParcelYolo::imageCb, this);
  yolo_client = nh_.serviceClient<parcel_msgs::parcel_poses>("parcel_find_server");
}

ParcelYolo::~ParcelYolo(){
  
}

void ParcelYolo::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  Image = *msg;
  cv_ptr->image.copyTo(opencvImage);
  Mat to_show;
  resize(opencvImage, to_show, cv::Size(720, 405), 0, 0, CV_INTER_NN);
  imshow(topic_name, to_show);
  waitKey(1);
}

bool ParcelYolo::find_parcel(){
  cout<<"SERVICE CALL"<<endl;
  parcel_list.request.Image = Image;
  if(yolo_client.call(parcel_list)){
    for(int i={}; i < parcel_list.response.parcel_pose.size(); i++){
      //your code
    }
  }
}

#endif