#ifndef OBJECT_CLUSTERING_H_
#define OBJECT_CLUSTERING_H_

#include "bin.hpp"
#include "object_builders/base_object_builder.hpp"
#include "object_builders/object_builder_manager.hpp"

namespace lidar {

class ObjectClustering {
public:
  ObjectClustering(double range_x_min, double range_x_max,
                   double range_y_min, double range_y_max,
                   double grid_res);

  void SegmentNonGroundPoints(lidar::PointICloudPtr &cloud,
                              std::vector<lidar::ObjectPtr> &objects);

private:
  bool Compare_z(int a, int b);
  double GetAngle(const lidar::PointI &point_);
  void ProjectPoints2Map(lidar::PointICloudPtr &cloud);
  void CheckGridMap(lidar::PointICloudPtr &cloud);
  int ConnectedComponentLabel(std::vector<int> &gridMapMatrix,
            std::vector<std::vector<int> > &connectedComponents);
  void FilterSmallComponents(std::vector<std::vector<int> > &connectedComponents);
  void ExtractClusters(const lidar::PointICloudPtr &cloud,
                       const std::vector<std::vector<int> > &connectedComponents,
                       std::vector<lidar::PointICloudPtr> &clusters);
  void HandleOverSegmentation(std::vector<lidar::ObjectPtr> &predictObjects,
                           std::vector<lidar::PointICloudPtr> &clusters);
  void HandleUnderSegmentation(std::vector<lidar::ObjectPtr> &predictObjects,
                               std::vector<lidar::PointICloudPtr> &clusters);
  void BuildMinBoxObjects(std::vector<lidar::PointICloudPtr> &clusters,
                    std::vector<lidar::ObjectPtr> &object);
  void RuleBasedObjectFilter(std::vector<lidar::ObjectPtr> &object);

private:
  double range_x_min_;
  double range_x_max_;
  double range_y_min_;
  double range_y_max_;
  double grid_res_;

  double height_diff_th_ = 0.2;
  double obj_height_gap_th_ = 0.5;
  size_t grid_point_num_th_ = 5;
  size_t cluster_point_num_th_ = 20;
  std::vector<Bin> grid_map_;
  std::vector<int> grid_map_matrix_;

  size_t row_size_;
  size_t col_size_;
};

}//namespace lidar

#endif //OBJECT_CLUSTERING_H_
