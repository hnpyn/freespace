#pragma once

#include <memory>
#include <string>

#include "Eigen/Core"
#include "common/types/object.hpp"
#include "common/types/trajectory.hpp"
#include "common/types/objects_tracked.hpp"
namespace lidar {

class MatchObjectDistance {
 public:
  // @brief set weight of location dist for all the track object distance
  // objects
  // @param[IN] location_distance_weight: weight of location dist
  // @return true if set successfully, otherwise return false
  static bool SetLocationDistanceWeight(const float location_distance_weight);

  // @brief set weight of bbox size dist for all the track object distance
  // objects
  // @param[IN] bbox_size_distance_weight: weight of bbox size dist
  // @return true if set successfully, otherwise return false
  static bool SetBboxSizeDistanceWeight(const float bbox_size_distance_weight);

  // @brief set weight of point num dist for all the track object distance
  // objects
  // @param[IN] point_num_distance_weight: weight of point num dist
  // @return true if set successfully, otherwise return false
  static bool SetPointNumDistanceWeight(const float point_num_distance_weight);

  // @brief compute distance for given cls_object & det_object
  // @param[IN] cls_object: clustering object
  // @param[IN] det_object: network detected object
  // @return computed <cls_object, det_object> distance
  static float ComputeDistance(
          const lidar::ObjectPtr& cls_object,
          const lidar::ObjectPtr& det_object);

  std::string Name() const { return "MatchObjectDistance"; }

 private:
  // @brief compute location distance for given cls_object & det_object
  // @param[IN] cls_object: clustering object
  // @param[IN] det_object: network detected object
  // @return computed location <cls_object, det_object> distance
  static float ComputeLocationDistance(
          const lidar::ObjectPtr& cls_object,
          const lidar::ObjectPtr& det_object);

  // @brief compute bbox size location distance for given cls_object & det_object
  // @param[IN] cls_object: clustering object
  // @param[IN] det_object: network detected object
  // @return computed bbox size location <cls_object, det_object> distance
  static float ComputeBboxSizeDistance(
          const lidar::ObjectPtr& cls_object,
          const lidar::ObjectPtr& det_object);

  static float ComputeBboxSizeDistanceV2(
          const lidar::ObjectPtr& cls_object,
          const lidar::ObjectPtr& det_object);

  // @brief compute point num distance for given track & object
  // @param[IN] track: track for <track, object> distance computing
  // @param[IN] new_object: recently detected object
  // @return point num distance of given <track, object>
  static float ComputePointNumDistance(
          const lidar::ObjectPtr& cls_object,
          const lidar::ObjectPtr& det_object);

 protected:
  // distance weights
  static double s_location_distance_weight_;
  static double s_bbox_size_distance_weight_;
  static double s_point_num_distance_weight_;

};  // class MatchObjectDistance

}  // namespace lidar

