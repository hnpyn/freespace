#include <ros/ros.h>
// PCL specific includes
#include <chrono>
#include <mutex>
#include <pcl/console/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <vector>

#include <config.h>
#include <eigen3/Eigen/Dense>
#include <free_space.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// std::clock_t startTime, endTime;
std::chrono::steady_clock::time_point startTime, endTime;
ros::Publisher pub_free_space;

std::mutex pose_mutex_;
nav_msgs::OdometryPtr input_pose_ptr_ = nullptr;
lidar::Point car_position_;
Eigen::Quaterniond car_orientation_;

void pose_callback(const nav_msgs::OdometryPtr &odom) {
  pose_mutex_.lock();
  input_pose_ptr_ = odom;
  pose_mutex_.unlock();
}

void parse_location_data(lidar::Point &car_pos,
                         Eigen::Quaterniond &car_orient) {
  if (input_pose_ptr_ == nullptr) {
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

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &point_cloud_msg) {
  config model_config("$HOME/freespace/src/"
                      "handle/config.yaml");
  // free space
  visualization_msgs::MarkerArray free_space_marker;
  sensor_msgs::PointCloud2 free_space_msg;
  lidarPerception::FreeSpace m_free_space;
  m_free_space.onPointCloud(point_cloud_msg, free_space_marker, free_space_msg,
                            model_config, roi_point);
}

int main(int argc, char **argv) {
  // Initialize ROS
  ros::init(argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("/fusion_points", 1, cloud_cb);
  ros::Subscriber pos_sub = nh.subscribe("/odom1", 1, &pose_callback);
  pub_free_space =
      nh.advertise<sensor_msgs::PointCloud2>("/free_space_cloud", 1);

  // Spin
  ros::spin();
}
