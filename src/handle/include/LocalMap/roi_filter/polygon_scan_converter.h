#ifndef MODULES_PERCEPTION_OBSTACLE_LIDAR_ROI_FILTER_HDMAP_ROI_FILTER_PSC_H_
#define MODULES_PERCEPTION_OBSTACLE_LIDAR_ROI_FILTER_HDMAP_ROI_FILTER_PSC_H_

#include <algorithm>
#include <limits>
#include <utility>
#include <vector>
#include "Eigen/Core"
#include "Eigen/StdVector"
//#include "gflags/gflags.h"

//#include "modules/common/log.h"
#include "bitmap2d.h"

namespace lidar {

/**
 * @class PolygonScanConverter
 * @brief: This is a converter from polygon to scan lines, by which we can build
 * bitmap. Assume major direction as x direction, we scan polygons in x
 * ascending
 * order.
 */
class PolygonScanConverter {
 public:
  typedef Eigen::Matrix<double, 2, 1> Point;
  typedef std::vector<Point, Eigen::aligned_allocator<Point>> Polygon;
  typedef std::pair<double, double> Interval;
  typedef std::pair<Point, Point> Segment;
  struct Edge {
    bool operator<(const Edge& other) const { return y < other.y; }
    bool MoveUp(const double delta_x);

    double max_x;
    double max_y;

    double x;
    double y;
    double k;
  };

  typedef Bitmap2D::DirectionMajor DirectionMajor;

  void Init(const DirectionMajor major_dir, const Interval& valid_range,
            const Polygon& polygon, const double step);

  static inline DirectionMajor OppositeDirection(DirectionMajor dir_major) {
    return static_cast<DirectionMajor>(dir_major ^ 1);
  }

  bool ConvertScans(std::vector<std::vector<Interval>>* scans_intervals);

 private:
  static const double kEpsilon;
  static const double kInf;
  Polygon polygon_;
  std::vector<Segment> segments_;

  std::vector<double> slope_;

  std::vector<std::vector<Edge>> edge_table_;

  std::vector<Edge> active_edge_table_;

  double min_x_;
  double step_;
  size_t scans_size_;
  DirectionMajor major_dir_;
  DirectionMajor op_major_dir_;

  /**
   * @brief: If some point of polygon happens to be around the scan line,
   * lightly modify the x coordinate to avoid situation of singular point.
   */
  void DisturbPolygon();

  void BuildEdgeTable();

  void UpdateActiveEdgeTable(const size_t x_id,
                             std::vector<Interval>* scan_intervals);

  void ConvertPolygonToSegments();

  bool ConvertSegmentToEdge(const size_t seg_id,
                            std::pair<int, Edge>* out_edge);
};

}  // namespace lidar

#endif  // MODULES_PERCEPTION_OBSTACLE_LIDAR_ROI_FILTER_HDMAP_ROI_FILTER_PSC_H_
