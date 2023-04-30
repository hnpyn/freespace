#ifndef CONFIG_H
#define CONFIG_H

#include <yaml-cpp/yaml.h>
#include <string>
#include <iostream>
#include <map>

class Config
{
public:
  Config() {}
  Config(const std::string &config_path);
  bool LoadConfig(const std::string &config_path);

public:
  // ros params
  std::string lidar_topic;
  std::string local_map_topic;
  std::string obj_marker_topic;
  std::string bg_cloud_topic;
  std::string ground_cloud_topic;
  std::string non_ground_cloud_topic;
  std::string clustering_region_topic;
  std::string pose_topic;

  // preprocessing params
  std::vector<float> ego_car_range;
  std::string local_map_file;

  // pointpillars params
  std::string pfe_model_file;
  std::string rpn_model_file;
  std::map<std::string, std::map<std::string, std::vector<float>>> anchor_config;
  std::vector<float> voxel_size;
  std::vector<float> point_cloud_range;
  int max_num_voxels;
  int max_points_per_voxel;
  int num_point_features;
  int num_gather_point_features;
  int num_pfe_output_channel;
  int num_thread;
  int num_class;
  int box_decode_size;
  float dir_offset;
  float dir_limit_offset;
  int num_dir_bins;
  float score_thresh;
  float nms_thresh;
  int nms_pre_maxsize;
  int nms_post_maxsize;

  // clustering params
  double range_x_min;
  double range_x_max;
  double range_y_min;
  double range_y_max;
  double grid_res;
};


#endif // CONFIG_H
