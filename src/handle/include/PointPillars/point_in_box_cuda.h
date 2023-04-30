#pragma once

#include <vector>
#include "cuda_common.h"

namespace lidar {
class PointInBoxCuda
{
public:
    PointInBoxCuda(float ratio = 1.0): ratio_(ratio) {}
    ~PointInBoxCuda() {}
    void Check(float *h_points, int n_points, float *boxes, int n_boxes, std::vector<int> &indice);
private:
    float ratio_;
};


} //namespace lidar
