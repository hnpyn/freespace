#ifndef MODULES_PERCEPTION_OBSTACLE_LIDAR_INTERFACE_HDMAP_ROI_FILTER_H_
#define MODULES_PERCEPTION_OBSTACLE_LIDAR_INTERFACE_HDMAP_ROI_FILTER_H_

#include <algorithm>
#include <string>
#include <vector>

#include "Eigen/Core"

//#include "modules/perception/proto/hdmap_roi_filter_config.pb.h"

//#include "modules/common/log.h"
//#include "perception_gflags.h"
#include "common/types/type.hpp"
#include "bitmap2d.h"
#include "polygon_mask.h"
#include "polygon_scan_converter.h"

namespace lidar {

typedef typename Bitmap2D::DirectionMajor MajorDirection;

/**
 * @class HdmapROIFilter
 * @brief This is ROI(Region of Interest) Filter based on HD map, which can
 * figure out which point is in the regions what we focus on(Almost in the
 * road).
 * This is an optional process, the most advantage of ROI filter is to filter
 * out some points to reduce calculation in the following process.
 */
class HdmapROIFilter {
 public:
  HdmapROIFilter():range_(50.0), cell_size_(0.25), extend_dist_(0.0){}
  ~HdmapROIFilter() {}

  std::string name() const { return "HdmapROIFilter"; }

  /**
   * @params[In] cloud: All the cloud points with local coordinates
   * @params[In] roi_filter_options: Type definition in
   * "../../interface/base_roi_filter.h". Contains the information
   * of ROI and world to local coordinates transformation matrix.
   * @params[Out] roi_indices: The indices of points within ROI
   * @return true if filter points successfully, otherwise return false
   */
  bool Filter(lidar::PointICloudPtr &cloud,
              std::vector<PolygonDType> &polygons,
              const Eigen::Matrix4d &transform,
              lidar::PointIndices &roi_indices,
              lidar::PointIndices &bg_indices) ;

 protected:
  /**
   * @brief: Draw polygons into grids in bitmap and check each point whether
   * is in the grids within ROI.
   */
  bool FilterWithPolygonMask(lidar::PointICloudPtr &cloud,
                             const std::vector<PolygonType>& map_polygons,
                             lidar::PointIndices &roi_indices,
                             lidar::PointIndices &bg_indices);

  /**
   * @brief: Transform polygon points from world coordinates
   * system to local.
   */
  void TransformFrame(const Eigen::Affine3d& vel_pose,
                      const std::vector<PolygonDType>& polygons_world,
                      std::vector<PolygonType>* polygons_local);

  /**
   * @brief: Get major direction. Transform polygons type to what we want.
   */
  MajorDirection GetMajorDirection(
      const std::vector<PolygonType>& map_polygons,
      std::vector<PolygonScanConverter::Polygon>* polygons);

  /**
   * @brief: After drawing polygons into grids in bitmap. We check each point
   * whether is in the grids within ROI.
   */
  bool Bitmap2dFilter(
      const pcl::PointCloud<lidar::PointI>::ConstPtr in_cloud_ptr,
      const Bitmap2D& bitmap, lidar::PointIndices &roi_indices,
      lidar::PointIndices &bg_indices);

  // We only filter point with local coordinates x, y in [-range, range] in
  // meters
  double range_ = 0.0;

  // Hight and width of grid in bitmap
  double cell_size_ = 0.0;

  // The distance extended away from the ROI boundary
  double extend_dist_ = 0.0;

};

}  // namespace lidar

#endif  // MODULES_PERCEPTION_OBSTACLE_LIDAR_INTERFACE_HDMAP_ROI_FILTER_H_
