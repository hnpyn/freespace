#ifndef MODULES_PERCEPTION_OBSTACLE_TRACK_OBJECT_DISTANCE_H_
#define MODULES_PERCEPTION_OBSTACLE_TRACK_OBJECT_DISTANCE_H_

#include <memory>
#include <string>

#include "Eigen/Core"
#include "common/types/trajectory.hpp"
#include "common/types/objects_tracked.hpp"

namespace lidar {

class TrackObjectDistance {
 public:
  // @brief set weight of location dist for all the track object distance
  // objects
  // @param[IN] location_distance_weight: weight of location dist
  // @return true if set successfully, otherwise return false
  static bool SetLocationDistanceWeight(const float location_distance_weight);

  // @brief set weight of direction dist for all the track object distance
  // objects
  // @param[IN] direction_distance_weight: weight of direction dist
  // @return true if set successfully, otherwise return false
  static bool SetDirectionDistanceWeight(const float direction_distance_weight);

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

  // @brief set weight of histogram dist for all the track object distance
  // objects
  // @param[IN] weight_histogram_dist: weight of histogram dist
  // @return true if set successfully, otherwise return false
  static bool SetHistogramDistanceWeight(const float histogram_distance_weight);

  // @brief compute distance for given track & object
  // @param[IN] track: track for <track, object> distance computing
  // @param[IN] track_predict: predicted state of given track
  // @param[IN] new_object: recently detected object
  // @return computed <track, object> distance
  static float ComputeDistance(
      trajectoryPtr track, const Eigen::VectorXf& track_predict,
      const std::shared_ptr<objects_tracked>& new_object);

  std::string Name() const { return "TrackObjectDistance"; }

 private:
  // @brief compute location distance for given track & object
  // @param[IN] track: track for <track, object> distance computing
  // @param[IN] track_predict: predicted state of given track
  // @param[IN] new_object: recently detected object
  // @return location distance of given <track, object>
  static float ComputeLocationDistance(
      trajectoryPtr track, const Eigen::VectorXf& track_predict,
      const std::shared_ptr<objects_tracked>& new_object);

  // @brief compute direction distance for given track & object
  // @param[IN] track: track for <track, object> distance computing
  // @param[IN] track_predict: predicted state of given track
  // @param[IN] new_object: recently detected object
  // @return direction distance of given <track, object>
  static float ComputeDirectionDistance(
      trajectoryPtr track, const Eigen::VectorXf& track_predict,
      const std::shared_ptr<objects_tracked>& new_object);

  // @brief compute bbox size distance for given track & object
  // @param[IN] track: track for <track, object> distance computing
  // @param[IN] new_object: recently detected object
  // @return bbox size distance of given <track, object>
  static float ComputeBboxSizeDistance(
      trajectoryPtr track, const std::shared_ptr<objects_tracked>& new_object);

  // @brief compute point num distance for given track & object
  // @param[IN] track: track for <track, object> distance computing
  // @param[IN] new_object: recently detected object
  // @return point num distance of given <track, object>
  static float ComputePointNumDistance(
      trajectoryPtr track, const std::shared_ptr<objects_tracked>& new_object);

  // @brief compute histogram distance for given track & object
  // @param[IN] track: track for <track, object> distance computing
  // @param[IN] new_object: recently detected object
  // @return histogram distance of given <track, object>
  static float ComputeHistogramDistance(
      trajectoryPtr track, const std::shared_ptr<objects_tracked>& new_object);

 protected:
  // distance weights
  static double s_location_distance_weight_;
  static double s_direction_distance_weight_;
  static double s_bbox_size_distance_weight_;
  static double s_point_num_distance_weight_;
  static double s_histogram_distance_weight_;

};  // class TrackObjectDistance

}  // namespace lidar


#endif  // MODULES_PERCEPTION_OBSTACLE_TRACK_OBJECT_DISTANCE_H_
