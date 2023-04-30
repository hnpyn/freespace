#ifndef POINTPILLARS_H
#define POINTPILLARS_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// headers in TensorRT
#include "NvInfer.h"
#include "NvOnnxParser.h"

#include "common/types/object.hpp"
#include "common/geometry_util.hpp"
#include "common/config.hpp"
#include "cuda_common.h"
#include "anchor_generator.h"
#include "preprocess_cuda.h"
#include "scatter_cuda.h"
#include "postprocess_cuda.h"
#include "point_in_box_cuda.h"

namespace lidar {

class PointPillars
{
public:
  PointPillars();
  ~PointPillars();
  bool Init(const Config &config);
  void Inference(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);
  void GetObjects(std::vector<lidar::ObjectPtr> &objects);

private:
  void LoadParams(const Config &config);
  void DeviceMemoryMalloc();
  void InitAnchors();
  void InitTRT();
  void OnnxToTRTModel(const std::string& model_file,
                      nvinfer1::ICudaEngine** engine_ptr);
  void CloudToArray(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud,
                    float* out_points_array);
  void Preprocess(const float* in_points_array, const int in_num_points);

  // parameters
  const int NUM_ANCHORS_PER_LOCATION_ = 2;
  const int FEATURE_MAP_SIZE_X_ = 300;
  const int FEATURE_MAP_SIZE_Y_ = 160;
  std::string pfe_onnx_file_;
  std::string rpn_onnx_file_;
  std::map<std::string, std::map<std::string, std::vector<float>>> anchor_config_;
  std::vector<float> voxel_size_;
  std::vector<float> point_cloud_range_;
  std::vector<int> grid_map_size_;

  int max_num_voxels_;
  int max_points_per_voxel_;
  int num_point_features_;
  int num_gather_point_features_;
  int num_pfe_output_channel_;
  int num_thread_;
  int num_class_;
  int box_decode_size_;
  float dir_offset_;
  float dir_limit_offset_;
  int num_dir_bins_;
  float score_thresh_;
  float nms_thresh_;
  int nms_pre_maxsize_;
  int nms_post_maxsize_;

  int pfe_output_size_;
  int rpn_cls_output_size_;
  int rpn_box_output_size_;
  int rpn_dir_output_size_;
  int num_anchors_;

  // cuda tensors
  float* dev_anchors_;
  int* dev_num_points_per_voxel_;
  float* dev_voxel_point_feature_;
  int* dev_voxel_coors_;
  float* dev_pfe_gather_feature_;    // extend voxel features, dim 10

  void* pfe_buffers_[2];
  void* rpn_buffers_[4];
  float* dev_scattered_feature_;

  int host_voxel_count_[1];
  int* dev_filter_count_;            // for postprocessing
  float* dev_filtered_box_;
  float* dev_filtered_score_;
  int* dev_filtered_label_;

  Logger g_logger_;
  nvinfer1::ICudaEngine* pfe_engine_;
  nvinfer1::ICudaEngine* rpn_engine_;
  nvinfer1::IExecutionContext* pfe_context_;
  nvinfer1::IExecutionContext* rpn_context_;

  std::unique_ptr<AnchorGenerator> anchor_generator_ptr_;
  std::unique_ptr<PreprocessorCuda> preprocessor_ptr_;
  std::unique_ptr<ScatterCuda> scatter_ptr_;
  std::unique_ptr<PostprocessCuda> postprocessor_ptr_;
  std::unique_ptr<PointInBoxCuda> point_in_box_checker_ptr_;

  std::vector<float> out_boxes_;
  std::vector<int> out_labels_;
  std::vector<float> out_scores_;
  std::vector<int> point_in_box_idx_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_ptr_;
};

}  // namespace lidar

#endif // POINTPILLARS_H
