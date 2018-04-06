//reference: ttbot - map_gen.cpp and rl_map.cpp
#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <fstream>
//#include <chrono>
#include <string>
#include <signal.h>
#include <math.h>
#include "deque"

#include <ros/ros.h>
#include <ros/package.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <ctime>
//#include <thread>
//TODO: Define and set core_msgs and their msg files listed here
#include "core_msgs/Vector3DArray.h"
#include "core_msgs/ROIPointArray.h"
#include "core_msgs/LaneInfo.h"
#include "core_msgs/MapFlag.h"
#include "core_msgs/CompressedImagePoint.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/Image.h"

#include "opencv2/opencv.hpp"
#include <boost/thread.hpp>

#define min(a,b) ((a)<(b)?(a):(b))

std::string config_path;


bool flag_imshow = true;
bool flag_record = true;

cv::VideoWriter outputVideo;

float lane_width;
int map_width, map_height, paddingx, paddingy;
float map_resol;

ros::Publisher publishMap;
core_msgs::MapFlagPtr msgMap;
image_transport::Publisher publishMapImg;
sensor_msgs::ImagePtr msgMapImg;
//cv_bridge::CvImagePtr cv_ptr;

boost::mutex map_mutex_;
std::vector<geometry_msgs::Vector3> obstacle_points;
int lidar_count=0;
float lidar_angle_min = 0;
float lidar_angle_increment = 0.001;

using namespace std;

bool check_out(int cx, int cy)
{
    return (cx<map_width-1)&&(cx>0)&&(cy<map_height-1)&&(cy>0);
}


//TODO: usage of mapInit()
void mapInit(cv::FileStorage& params_config) {
  map_width = params_config["Map.width"];
  map_height = params_config["Map.height"];
  map_resol = params_config["Map.resolution"];
  paddingx = params_config["Map.obstacle.paddingx"];
  paddingy = params_config["Map.obstacle.paddingy"];
}

//TODO: callbackLane
void callbackLane(const core_msgs::LaneInfoConstPtr& lane_info)
{
  //TODO: replace LanePlaceholder with Lane obj
  //drawLane(lane_info, map);
}


//TODO:
void drawLane(const core_msgs::LaneInfoConstPtr& lane_info_, cv::Mat& map) {
}

int drawObstaclePoints(std::vector<geometry_msgs::Vector3>& _obstacle_points, cv::Mat& map) {
  float obstacle_x, obstacle_y;
  int cx, cy;
  int cx1, cx2, cy1, cy2;
  int obstacle_count = 0;

  for(int i= _obstacle_points.size()-1; i>=0;i--){
    obstacle_x = _obstacle_points.at(i).x*cos(_obstacle_points.at(i).y);
    obstacle_y = _obstacle_points.at(i).x*sin(_obstacle_points.at(i).y);

    cx = map_width/2 + (int)(obstacle_x/map_resol);
    cy = map_height - (int)(obstacle_y/map_resol);

    if(check_out(cx, cy)){
      obstacle_count++;
      if(check_out(cx-paddingx, cy-paddingy)){
        if(check_out(cx+paddingx,cy+paddingy)){
          cx1 = cx-paddingx;
          cy1 = cy-paddingy;
          cx2 = cx+paddingx;
          cy2 = cy+paddingy;
        }else {
          cx1 = cx-paddingx;
          cy1 = cy-paddingy;
          cx2 = cx; cy2 = cy;
        }
      } else if(check_out(cx+paddingx,cy+paddingy)) {
        cx1 = cx; cy1 = cy;
        cx2 = cx+paddingx;
        cy2 = cy+paddingy;
      } else {
        cx1 = cx; cy1 = cy;
        cx2 = cx; cy2 = cy;
      }
      cv::rectangle(map,cv::Point(cx1, cy1), cv::Point(cx2, cy2), cv::Scalar(255,255,255), -1);
    }
  }
  return obstacle_count;
}

//TODO:
//Map 은 라이더 위치를 Global Frame에서 (0,0), Img Frame에서 (map_width/2, map_height)로 보고 작성
//따라서 Lane 정보를 받게 되면 x 방향으로 무게중심이랑 이미지 frame 거리 차만큼 빼줘야 함
void drawObstacle() {
    cv::Mat map = cv::Mat::zeros(map_width,map_height,CV_8UC3);
    int flag_obstacle = 0;

    //flag_obstacle is 1 if there are any obstacle within the lane

    flag_obstacle = drawObstaclePoints(obstacle_points, map);


    //publishMap
    msgMap->header.stamp = ros::Time::now();
    msgMap->frame = *(cv_bridge::CvImage(std_msgs::Header(),"rgb8", map).toImageMsg());
    msgMap->flag = flag_obstacle;
    publishMap.publish(msgMap);

    //cv_ptr = cv_bridge::toCvShare(map, sensor_msgs::image_encodings::BGR8);
    msgMapImg = cv_bridge::CvImage(std_msgs::Header(),"rgb8", map).toImageMsg();
    publishMapImg.publish(msgMapImg);

    //if(flag_record) {
      cv::Mat map_resized = cv::Mat::zeros(500,500,CV_8UC3);
      cv::resize(map, map_resized, cv::Size(500,500),0,0,CV_INTER_NN);
      outputVideo << map_resized;
    //}
}

//TODO: callbackObstacle
void callbackObstacle(const core_msgs::ROIPointArrayConstPtr& msg_obstacle)
{
  //TODO: CHOI? map_mutex_??
  map_mutex_.lock();
  obstacle_points = msg_obstacle->Vector3DArray;
  lidar_count = msg_obstacle->id[0];
  lidar_angle_min = msg_obstacle->extra[0];
  lidar_angle_increment = msg_obstacle->extra[1];
  map_mutex_.unlock();

  drawObstacle();
}


void callbackTerminate(const std_msgs::Int32Ptr& record){
  //if(flag_record){
    outputVideo.release();
  //}

  cv::Mat map = cv::Mat::zeros(map_width,map_height,CV_8UC3);
  drawObstaclePoints(obstacle_points, map);
  std::string path = ros::package::getPath("map_generator");
  path += "/../../../data/test_data/";
  cv::Mat map_resized = cv::Mat::zeros(800,800,CV_8UC3);
  cv::resize(map, map_resized, cv::Size(800,800), 0,0, CV_INTER_NN);
  cv::imwrite(path+"map.png",map_resized);

  ROS_INFO("Video recording safely terminated");
  ros::shutdown();
  return;
}


//argv: 1:nodeName 2:flag_imshow 3:flag_record(true if you are to record video)
//CHOI? flag_imshow의 usage?
int main(int argc, char** argv)
{
  //argument setting initialization
  if(argc < 3)  {
      std::cout << "usage: rosrun map_generator map_generator_node flag_imshow flag_record" << std::endl;
      return -1;
  }

  if (!strcmp(argv[1], "false")) flag_imshow = false;
  if (!strcmp(argv[2], "false")) flag_record = false;
  std::string record_path = ros::package::getPath("map_generator");
  record_path += "/data/map.avi";
  ROS_INFO_STREAM(record_path);
  //if(flag_record) {
  bool isVideoOpened =  outputVideo.open(record_path, CV_FOURCC('X', 'V', 'I', 'D'), 25, cv::Size(500,500), true);
  //}
  if(isVideoOpened)
    ROS_INFO("video starts recorded!");

  //TODO: change root path later
  config_path = ros::package::getPath("map_generator");
  config_path += "/config/system_config.yaml";
  cv::FileStorage params_config(config_path, cv::FileStorage::READ);
  lane_width = params_config["Road.lanewidth"];

  mapInit(params_config);

  ros::init(argc, argv, "map_generator");
  ros::start();

  // /map publish를 위한 설정 (publishMap & msgMap)
  ros::NodeHandle nh;
  publishMap = nh.advertise<core_msgs::MapFlag>("/map",1);
  msgMap.reset(new core_msgs::MapFlag);
  image_transport::ImageTransport it(nh);

  publishMapImg = it.advertise("/map_img",1);
  msgMapImg.reset(new sensor_msgs::Image);


  // /lane_info subscribe를 위한 설정
  //ros::Subscriber laneSub = nh.subscribe("/lane_info",1,callbackLane);

  // /obstacle_points subscribe를 위한 설정
  // 해당 topic을 받을 때마다 callbackObstacle이 실행됨
  ros::Subscriber obstacleSub = nh.subscribe("/obstacle_points",1,callbackObstacle);
  ros::Subscriber endSub = nh.subscribe("/end_system",1,callbackTerminate);
  ros::spin();
  return 0;
}
