#ifndef MODULES_PERCEPTION_OBSTACLE_LIDAR_ROI_FILTER_HDMAP_ROI_FILTER_BM_H_
#define MODULES_PERCEPTION_OBSTACLE_LIDAR_ROI_FILTER_HDMAP_ROI_FILTER_BM_H_

#include <limits>
#include <vector>

#include "Eigen/Core"

//#include "modules/common/log.h"

namespace lidar {

/**
 * @class Bitmap2D
 * @brief This is Bitmap, which store the ROI information as grids. If the value
 * of grid is true, means this grid is in ROI, otherwise means the grid is out
 * of ROI.
 *
 * @Note: In column direction, each bit denotes one grid. To speed up range set
 * operation, we use uint64_t to represent 64 grids, which can set 64 gird at
 * one time.
 */
class Bitmap2D {
 public:
  enum DirectionMajor { XMAJOR = 0, YMAJOR = 1 };
  Bitmap2D(const Eigen::Vector2d& min_p, const Eigen::Vector2d& max_p,
           const Eigen::Vector2d& grid_size, DirectionMajor dir_major);

  static inline DirectionMajor opposite_direction(DirectionMajor dir_major) {
    return static_cast<DirectionMajor>(dir_major ^ 1);
  }

  const Eigen::Vector2d& get_min_p() const { return min_p_; }
  const Eigen::Vector2d& get_max_p() const { return max_p_; }
  const Eigen::Vector2d& get_grid_size() const { return grid_size_; }
  const DirectionMajor get_dir_major() const { return dir_major_; }
  const DirectionMajor get_op_dir_major() const { return op_dir_major_; }

  /**
   * @brief: Roughly check whether the point is in bitmap.
   */
  bool IsExist(const Eigen::Vector2d& p) const;

  /**
   * @brief: Cast point to a grid in bitmap, then check whether the value of
   * gird is true(in ROI).
   */
  bool Check(const Eigen::Vector2d& p) const;

  void Set(double x, double min_y, double max_y);
  void Set(const uint64_t x_id, const uint64_t min_y_id,
           const uint64_t max_y_id);

  void BuildMap();

 private:
  Eigen::Vector2d min_p_;
  Eigen::Vector2d max_p_;
  Eigen::Vector2d grid_size_;
  DirectionMajor dir_major_;
  DirectionMajor op_dir_major_;

  std::vector<std::vector<uint64_t>> bitmap_;

  inline void SetUint64RangeBits(const size_t head, const size_t tail,
                                 uint64_t* block);
  inline void SetUint64HeadBits(const size_t head, uint64_t* block);
  inline void SetUint64TailBits(const size_t tail, uint64_t* block);
};

}  // namespace perception

#endif  // MODULES_PERCEPTION_OBSTACLE_LIDAR_ROI_FILTER_HDMAP_ROI_FILTER_BM_H_
