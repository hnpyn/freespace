#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/time.h>
#include <string>
#include <mutex>
#include <vector>
#include <chrono>

#include <eigen3/Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <free_space.h>
#include <config.h>
#include "LocalMap/local_map_filter.h"


// std::clock_t startTime, endTime;
std::chrono::steady_clock::time_point startTime, endTime;
ros::Publisher pub_free_space_marker, pub_free_space, pub_local_map_roi;

string txtFilePath = "/home/wang/driveworks/map/local_map/local_map_v1.txt";
std::mutex pose_mutex_;
nav_msgs::OdometryPtr input_pose_ptr_ = nullptr;
lidar::Point car_position_;
Eigen::Quaterniond car_orientation_;

void
pose_callback(const nav_msgs::OdometryPtr &odom)
{
  pose_mutex_.lock();
  input_pose_ptr_ = odom;
  pose_mutex_.unlock();
}

void parse_location_data(lidar::Point &car_pos,
                                        Eigen::Quaterniond &car_orient)
{
  if (input_pose_ptr_ == nullptr){
    return;
  }
  car_pos.x = input_pose_ptr_->pose.pose.position.x;
  car_pos.y = input_pose_ptr_->pose.pose.position.y;
  car_pos.z = input_pose_ptr_->pose.pose.position.z - 1.8;

  car_orient.x() = input_pose_ptr_->pose.pose.orientation.x;
  car_orient.y() = input_pose_ptr_->pose.pose.orientation.y;
  car_orient.z() = input_pose_ptr_->pose.pose.orientation.z;
  car_orient.w() = input_pose_ptr_->pose.pose.orientation.w;
}

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg)
{
  config model_config("/home/wang/driveworks/freespace/freespace_lidar_v3/src/handle/config.yaml");
  
  // local map ROI
  lidar::LocalMapFilter local_map_filter;
  lidar::PointICloudPtr bg_cloud_ptr(new lidar::PointICloud);
  lidar::PointICloudPtr point_cloud_(new lidar::PointICloud);
  lidar::RoiPoint roi_point;
  startTime = std::chrono::steady_clock::now();
  parse_location_data(car_position_, car_orientation_);
  pcl::fromROSMsg(*point_cloud_msg, *point_cloud_);
  endTime = std::chrono::steady_clock::now();
  std::cout << "sss " << std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count()  
                    << std::endl;
  local_map_filter.LoadLocalMap(txtFilePath);
  local_map_filter.Filter(point_cloud_, bg_cloud_ptr, car_position_, car_orientation_);
  local_map_filter.PublishPolygons(pub_local_map_roi, point_cloud_msg->header);
  local_map_filter.CalculateLine(roi_point);

  // free space
  visualization_msgs::MarkerArray free_space_marker;
  sensor_msgs::PointCloud2 free_space_msg;
  //lidarPerception::main_perception *perceptionPtr = new lidarPerception::main_perception;
  lidarPerception::FreeSpace m_free_space;
  m_free_space.onPointCloud(point_cloud_msg, free_space_marker, free_space_msg, model_config, roi_point);
  pub_free_space_marker.publish(free_space_marker);
  pub_free_space.publish(free_space_msg);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  //ros::Subscriber sub = nh.subscribe ("/pandar_points", 1, cloud_cb);
  ros::Subscriber sub = nh.subscribe ("/fusion_points", 1, cloud_cb);
  ros::Subscriber pos_sub = nh.subscribe("/odom1", 1, &pose_callback);
  //ros::Subscriber sub = nh.subscribe ("/adjust", 1, cloud_cb);
  // Create a ROS publisher for the output point cloud
  pub_free_space_marker = nh.advertise<visualization_msgs::MarkerArray>("/free_space1", 1);
  pub_free_space = nh.advertise<sensor_msgs::PointCloud2> ("/free_space_cloud", 1);
  // local_map_roi
  pub_local_map_roi = nh.advertise<visualization_msgs::MarkerArray>("local_map_roi", 1);
  
  // Spin
  ros::spin();
}
