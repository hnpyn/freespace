#ifndef LOCAL_MAP_FILTER_H
#define LOCAL_MAP_FILTER_H

#include <string>
#include <vector>
#include <queue>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include "common/types/type.hpp"
#include "LocalMap/roi_filter/roi_filter.h"

namespace lidar {

struct roipoint
{
  roipoint()
  {
    x = 0.0;
    y = 0.0;
    z = 0.0;
    ag = 0.0;
    dis = 0.0;
  }
    double x;
    double y;
    double z;
    double ag;
    double dis;
};
typedef std::vector<roipoint> RoiPoint;
typedef std::queue<roipoint> RoiQueue;

class LocalMapFilter
{
public:
  LocalMapFilter();
  void LoadLocalMap(const std::string &file_name);
  void Filter(lidar::PointICloudPtr &cloud,
              lidar::PointICloudPtr &bg_cloud,
              const lidar::Point &location,
              const Eigen::Quaterniond &quaternion);
  void PublishPolygons(const ros::Publisher &pub, const std_msgs::Header &header);
  void CalculateLine(lidar::RoiPoint &roiPoint);

private:
  std::vector<lidar::PolygonDType> polygons;
  std::vector<lidar::PolygonDType> selectedPolygons;
  std::vector<lidar::PolygonDType> selectedOnePolygons;
  Eigen::Matrix4d transform;

private:
  void SearchPolygons(const lidar::Point &pos);
  void SetTransform(const lidar::Point &location, const Eigen::Quaterniond &quaternion);
  void ExtractFilterCloud(lidar::PointICloudPtr &raw_cloud, lidar::PointICloudPtr &filter_cloud,
                          lidar::PointIndices &indices);
};

} //namespace lidar
#endif // LOCAL_MAP_FILTER_H
