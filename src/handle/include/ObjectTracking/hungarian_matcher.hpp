#ifndef MODULES_PERCEPTION_LIDAR_TRACKER_HM_TRACKER_HUNGARIAN_MATCHER_H_
#define MODULES_PERCEPTION_LIDAR_TRACKER_HM_TRACKER_HUNGARIAN_MATCHER_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ObjectTracking/interface/base_matcher.hpp"
#include "track_object_distance.hpp"
#include "common/graph_util.hpp"

namespace lidar {

class HungarianMatcher : public baseMatcher {
 public:
  HungarianMatcher() {}
  ~HungarianMatcher() {}

  // @brief set match distance maximum for matcher
  // @param[IN] match_distance_maximum: match distance maximum
  // @return true if set successfully, otherwise return false
  static bool SetMatchDistanceMaximum(const float match_distance_maximum);

  // @brief match detected objects to tracks
  // @param[IN] objects: new detected objects for matching
  // @param[IN] tracks: maintaining tracks for matching
  // @param[IN] tracks_predict: predicted state of maintained tracks
  // @param[OUT] assignments: assignment pair of object & track
  // @param[OUT] unassigned_tracks: tracks without matched object
  // @param[OUT] unassigned_objects: objects without matched track
  // @return nothing
  void Match(std::vector<std::shared_ptr<objects_tracked>>* objects,
             const std::vector<trajectoryPtr>& tracks,
             const std::vector<Eigen::VectorXf>& tracks_predict,
             std::vector<std::pair<int, int>>* assignments,
             std::vector<int>* unassigned_tracks,
             std::vector<int>* unassigned_objects);

  // @brief match detected objects to tracks in component level
  // @param[IN] association_mat: association matrix of all objects to tracks
  // @param[IN] track_component: component of track
  // @param[IN] object_component: component of object
  // @param[OUT] sub_assignments: component assignment pair of object & track
  // @param[OUT] sub_unassigned_tracks: component tracks not matched
  // @param[OUT] sub_unassgined_objects: component objects not matched
  // @return nothing
  void MatchInComponents(const Eigen::MatrixXf& association_mat,
                         const std::vector<int>& track_component,
                         const std::vector<int>& obj_component,
                         std::vector<std::pair<int, int>>* sub_assignments,
                         std::vector<int>* sub_unassigned_tracks,
                         std::vector<int>* sub_unassigned_objects);

  std::string Name() const { return "HungarianMatcher"; }

 protected:
  // @brief compute association matrix
  // @param[IN] tracks: maintained tracks for matching
  // @param[IN] tracks_predict: predicted states of maintained tracks
  // @param[IN] new_objects: recently detected objects
  // @param[OUT] association_mat: matrix of association distance
  // @return nothing
  void ComputeAssociateMatrix(
      const std::vector<trajectoryPtr>& tracks,
      const std::vector<Eigen::VectorXf>& tracks_predict,
      const std::vector<std::shared_ptr<objects_tracked>>& new_objects,
      Eigen::MatrixXf* association_mat);

  // @brief compute connected components within given threshold
  // @param[IN] association_mat: matrix of association distance
  // @param[IN] connected_threshold: threshold of connected components
  // @param[OUT] track_components: connected objects of given tracks
  // @param[OUT] obj_components: connected tracks of given objects
  // @return nothing
  void ComputeConnectedComponents(
      const Eigen::MatrixXf& association_mat, const float connected_threshold,
      std::vector<std::vector<int>>* track_components,
      std::vector<std::vector<int>>* obj_components);

  // @brief assign objects to tracks using components
  // @param[IN] association_mat: matrix of association distance
  // @param[IN] assign_distance_maximum: threshold distance of assignment
  // @param[OUT] assignments: assignment pair of matched object & track
  // @param[OUT] unassigned_tracks: tracks without matched object
  // @param[OUT] unassigned_objects: objects without matched track
  // @return nothing
  void AssignObjectsToTracks(const Eigen::MatrixXf& association_mat,
                             const double assign_distance_maximum,
                             std::vector<std::pair<int, int>>* assignments,
                             std::vector<int>* unassigned_tracks,
                             std::vector<int>* unassigned_objects);

 private:
  // threshold of matching
  static float s_match_distance_maximum_;

};  // class HmMatcher

}  // namespace lidar

#endif  // MODULES_PERCEPTION_LIDAR_TRACKER_HM_TRACKER_HUNGARIAN_MATCHER_H_
