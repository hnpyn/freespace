#ifndef MODULES_PERCEPTION_OBSTACLE_LIDAR_ROI_FILTER_HDMAP_ROI_FILTER_PM_H_
#define MODULES_PERCEPTION_OBSTACLE_LIDAR_ROI_FILTER_HDMAP_ROI_FILTER_PM_H_

#include <algorithm>
#include <limits>
#include <vector>

#include "Eigen/StdVector"

//#include "modules/common/log.h"
#include "bitmap2d.h"
#include "polygon_scan_converter.h"

namespace lidar {

typedef typename PolygonScanConverter::Interval Interval;

void DrawPolygonInBitmap(const PolygonScanConverter::Polygon& polygon,
                         const double extend_dist, Bitmap2D* bitmap);

void DrawPolygonInBitmap(
    const std::vector<PolygonScanConverter::Polygon>& polygons,
    const double extend_dist, Bitmap2D* bitmap);

/*
 * @brief: Get valid x range(Major direction range)
 */
bool GetValidXRange(const PolygonScanConverter::Polygon& polygon,
                    const Bitmap2D& bitmap,
                    const PolygonScanConverter::DirectionMajor major_dir,
                    const double major_dir_grid_size, Interval* valid_x_range);

}  // namespace lidar

#endif  // MODULES_PERCEPTION_OBSTACLE_LIDAR_ROI_FILTER_HDMAP_ROI_FILTER_PM_H_
