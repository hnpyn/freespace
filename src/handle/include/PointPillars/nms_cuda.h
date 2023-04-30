#pragma once

// headers in STL
#include <vector>
#include "cuda_common.h"

namespace lidar {

class NmsCuda {
public:
  NmsCuda(const int num_threads, const float nms_overlap_thresh);
  void DoNmsCuda(const int num_boxes, float* dev_sorted_box_for_nms,
                 int* out_keep_inds, int* out_num_to_keep);

private:
  const int num_threads_;
  const float nms_overlap_thresh_;
};

} // namespace lidar
