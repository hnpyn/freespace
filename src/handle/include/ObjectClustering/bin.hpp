#ifndef GROUND_SEGMENTATION_BIN_H_
#define GROUND_SEGMENTATION_BIN_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

namespace lidar {

class Bin {
public:
  Bin():min_height(100.0), max_height(-100.0){}
  void AddPoint(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, int idx);

public:
  std::vector<int> point_idx;
  std::vector<double> point_z;

  double min_height;
  double max_height;
  double sum_height;

  enum{
    BIN_UNKNOWN = -1,
    BIN_FREE = 0,
    BIN_OBSTACLE = 1,
  }state;

};

}

#endif /* GROUND_SEGMENTATION_BIN_H_ */
