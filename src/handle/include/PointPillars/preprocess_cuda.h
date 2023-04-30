#pragma once
#include <vector>
#include "cuda_common.h"

namespace lidar {

class PreprocessorCuda {

public:
  /**
   * @brief Constructor
   * @param[in] num_threads Thread number for cuda kernel
   * @param[in] max_num_voxels Maximum number of voxels
   * @param[in] max_points_per_voxel Maximum number of points per voxel
   * @param[in] num_point_feature Number of features in a point
   * @param[in] voxel [x, y, z] Size for a pillar, eg. [0.2, 0.2, 6.0]
   * @param[in] point_cloud_range [x_min, y_min, z_min, x_max, y_max, z_max]
   */
  PreprocessorCuda(const int num_threads,
                   const int max_num_voxels,
                   const int max_points_per_voxel,
                   const int num_point_features,
                   const int num_pfe_gather_point_features,
                   const std::vector<float> voxel_size,
                   const std::vector<float> point_cloud_range,
                   const std::vector<int> grid_size);
  ~PreprocessorCuda();
  int Point2Voxels(const float* dev_points, const int n_points,
                   std::vector<float> &voxels_array,
                   std::vector<int> &voxels_num_points_array,
                   std::vector<int> &voxels_coords_array);
  /**
   * @brief CUDA preprocessing for input point cloud
   * @param[in] dev_points Point cloud array
   * @param[in] in_num_points The number of points
   * @param[in] dev_x_coors X-coordinate indexes for corresponding pillars
   * @param[in] dev_y_coors Y-coordinate indexes for corresponding pillars
   * @param[in] dev_num_points_per_pillar
   *   Number of points in corresponding pillars
   * @param[in] pillar_point_feature
   *   Values of point feature in each pillar
   * @param[in] pillar_coors Array for coors of pillars
   * @param[in] dev_sparse_pillar_map
   *   Grid map representation for pillar-occupancy
   * @param[in] host_pillar_count
   *   The number of valid pillars for an input point cloud
   * @details Convert point cloud to pillar representation
   */
  void DoPreprocessCuda(const float* dev_points, const int in_num_points,
                              int* dev_num_points_per_pillar,
                              float* dev_pillar_point_feature,
                              int* dev_pillar_coors,
                              int* host_pillar_count,
                              float* dev_pfe_gather_feature);
private:
  // initializer list
  const int num_threads_;
  const int max_num_voxels_;
  const int max_points_per_voxel_;
  const int num_point_features_;
  const int num_pfe_gather_point_features_;
  const std::vector<float> voxel_size_;
  const std::vector<float> point_cloud_range_;
  std::vector<int> grid_size_;
  // end initializer list

  float* dev_voxel_point_feature_in_coors_;
  int* dev_voxel_count_histo_;

  int* dev_counter_;
  int* dev_voxel_count_;

};

}  // namespace lidar
