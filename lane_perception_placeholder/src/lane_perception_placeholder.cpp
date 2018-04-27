#include <iostream>
#include <stdio.h>
#include <string>
#include <ros/ros.h>
#include <ros/package.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include "sensor_msgs/Image.h"
#include "opencv2/opencv.hpp"

int map_width, map_height;
std::string config_path;

int main(int argc, char ** argv) {
  ros::init(argc, argv, "lane_perception_placeholder");
  config_path = ros::package::getPath("map_generator");
  config_path += "/config/system_config.yaml";
  cv::FileStorage params_config(config_path, cv::FileStorage::READ);
  map_width = params_config["Map.width"];
  map_height = params_config["Map.height"];

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pubLane = it.advertise("/lane_map",1);;
  ros::Rate loop_rate(20);

  cv::Mat lane_map = cv::Mat::zeros(map_width, map_height, CV_8UC1);
  //cv::rectangle(lane_map, cv::Point(0,map_height-10), cv::Point(40,20),255, -1);
  //cv::rectangle(lane_map, cv::Point(map_width-40,map_height-10), cv::Point(map_width-1,20),255, -1);

  while(ros::ok())
  {
    sensor_msgs::ImagePtr lane_map_msg = cv_bridge::CvImage(std_msgs::Header(),"mono8", lane_map).toImageMsg();
    pubLane.publish(lane_map_msg);
  }
  return 0;
}
