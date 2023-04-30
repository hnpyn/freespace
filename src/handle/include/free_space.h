#ifndef FREE_SPACE_H
#define FREE_SPACE_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/console/time.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include "pcl/filters/radius_outlier_removal.h"
#include "LocalMap/local_map_filter.h"

#include <cmath>
//#include <math.h>
#include <ros/ros.h>

#include <config.h>

typedef pcl::PointXYZI PointI;
typedef pcl::PointCloud<PointI> PointIcloud;
typedef PointIcloud::Ptr PointICloudPtr;

namespace lidarPerception {
  
struct pointxy
{
    pointxy()
    {
        x = 0.;
        y = 0.;
    }
    float x;
    float y;
};


class FreeSpace
{
public:
  struct grid_feature
  {
    grid_feature() {
      maxHeight = -99.0;
      minHeight = 99.0;
      closestDis = 99.0;
      pointsNums = 0;
    }
    double maxHeight;
    double minHeight;
    double closestDis;
    int pointsNums;
  };
  std::vector<pointxy> fs_points;


public:
  FreeSpace();
  void get_free_space(const std_msgs::Header header, const PointICloudPtr &point_cloud, 
                    std::vector<int> &list, lidar::RoiPoint roi_point,
                    PointICloudPtr out_point_cloud, 
                    PointICloudPtr out_point_cloud_xy, 
                    visualization_msgs::MarkerArray &free_space_marker);
  void onPointCloud(const sensor_msgs::PointCloud2ConstPtr &point_cloud_msg,
                    visualization_msgs::MarkerArray &free_space_marker,
                    sensor_msgs::PointCloud2 &free_space_msg,const config fsconfig,
                    lidar::RoiPoint roi_point);
  void onPointCloud_wangran(const sensor_msgs::PointCloud2ConstPtr &point_cloud_msg,
                    sensor_msgs::PointCloud2 &free_space_msg);

private:
  bool construct_polar_grid(const PointICloudPtr &point_cloud,
                            std::vector<std::vector<grid_feature> > &polar_grid_points);
  double get_theta(const PointI &point);
  double get_theta(const int i);
  double get_points_distance(const PointI &point);
  void check_grid(const std::vector<std::vector<grid_feature> > &polar_grid_points, const int rows, const int cols,
                  std::vector<std::vector<int> > &objects_grid);
  void get_closest_distance(std::vector<std::vector<grid_feature> > &polar_grid_points, const std::vector<std::vector<int> > &objects_grid,
                            const int nRows, const int nCols, std::vector<double> &point_list);
  void get_closest_distance_2(std::vector<std::vector<grid_feature> > &polar_grid_points,
                              const std::vector<std::vector<int> > &objects_grid,
                              const int nRows, const int nCols, std::vector<double> &point_list);
  void get_point_position(const double theta, const double distance, PointI &out_point_cloud);
  void draw_points(const std::vector<double> &point_list, PointICloudPtr &out_point_cloud);
  void filter_points_cloud(const PointICloudPtr point_cloud, PointICloudPtr filter_point_cloud);
  void draw_lines(const std_msgs::Header header, PointICloudPtr out_point_cloud, visualization_msgs::MarkerArray &free_space_marker);
  void correct_points(std::vector<double> &point_list);
  void correct_points_angle(std::vector<double> &point_list);
  double calculate_points_dis(const PointI pa, const PointI pb);
  double calculate_dis(const PointI p);
  bool check_corrected_points(int i, std::vector<int> points_has_corrected);
  void shrink_angle(std::vector<double> &point_list, std::vector<double> &out_point_list);
  void get_point_list_xy(std::vector<double> &out_point_list,
                         std::vector<PointI> &out_point_list_xy, PointICloudPtr out_point_cloud_xy);
  void transform_formats(std::vector<PointI> &out_point_list_xy, std::vector<int> &list);
  void correct_freespace(lidar::RoiPoint roi_point, PointICloudPtr out_point_cloud, PointICloudPtr &temp_point_cloud);


private:
  config config_;

};
}


#endif // FREE_SPACE_H
