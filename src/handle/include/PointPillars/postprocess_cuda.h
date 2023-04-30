#pragma once

// headers in STL
#include <algorithm>

#include "common/config.hpp"
#include "cuda_common.h"
#include "nms_cuda.h"

namespace lidar {

class PostprocessCuda {
 public:
  /**
   * @brief Constructor
   * @param[in] float_min The lowest float value
   * @param[in] float_max The maximum float value
   * @param[in] num_anchor Number of anchors in total
   * @param[in] num_class Number of object's classes
   * @param[in] score_threshold Score threshold for filtering output
   * @param[in] num_threads Number of threads when launching cuda kernel
   * @param[in] nms_overlap_threshold IOU threshold for NMS
   * @param[in] num_box_corners Number of box's corner
   * @param[in] num_output_box_feature Number of output box's feature
   * @details Captital variables never change after the compile, non-capital
   * variables could be changed through rosparam
   */
  PostprocessCuda(const int num_anchor, const int num_class,
                  const int num_thread, const float score_th,
                  const float nms_overlap_th, const int box_decode_size,
                  const int num_dir_bins, const float dir_offset,
                  const float dir_limit_offset, const int nms_pre_maxsize, const int nms_post_maxsize);

  /**
   * @brief Postprocessing for the network output
   * @param[in] rpn_box_output Box predictions from the network output
   * @param[in] rpn_cls_output Class predictions from the network output
   * @param[in] rpn_dir_output Direction predictions from the network output
   * @param[in] dev_anchor_mask Anchor mask for filtering the network output
   * @param[in] dev_anchors_px X-coordinate values for corresponding anchors
   * @param[in] dev_anchors_py Y-coordinate values for corresponding anchors
   * @param[in] dev_anchors_pz Z-coordinate values for corresponding anchors
   * @param[in] dev_anchors_dx X-dimension values for corresponding anchors
   * @param[in] dev_anchors_dy Y-dimension values for corresponding anchors
   * @param[in] dev_anchors_dz Z-dimension values for corresponding anchors
   * @param[in] dev_anchors_ro Rotation values for corresponding anchors
   * @param[in] dev_filtered_box Filtered box predictions
   * @param[in] dev_filtered_score Filtered score predictions
   * @param[in] dev_filtered_label Filtered label predictions
   * @param[in] dev_filtered_dir Filtered direction predictions
   * @param[in] dev_box_for_nms Decoded boxes in min_x min_y max_x max_y
   * represenation from pose and dimension
   * @param[in] dev_filter_count The number of filtered output
   * @param[out] out_detection Output bounding boxes
   * @param[out] out_label Output labels of objects
   * @details dev_* represents device memory allocated variables
   */
  void DoPostprocessCuda(
      const float* rpn_cls_output, const float* rpn_box_output,
      const float* rpn_dir_output, const float* dev_anchors,
      float* dev_filtered_box, float* dev_filtered_score,
      int* dev_filtered_label, int* dev_filter_count,
      std::vector<float>& out_boxes,
      std::vector<int>& out_labels,
      std::vector<float>& out_scores);

private:
  const int num_anchor_;
  const int num_class_;
  const int num_threads_;
  const float score_threshold_;
  const float nms_overlap_threshold_;
  const int box_decode_size_;
  const int num_dir_bins_;
  const float dir_offset_;
  const float dir_limit_offset_;
  const int nms_pre_maxsize_;
  const int nms_post_maxsize_;

  std::unique_ptr<NmsCuda> nms_ptr_;
};

}  // namespace lidar
