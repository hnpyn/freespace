#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include "common/types/object.hpp"
#include "match_object_distance.hpp"
#include "common/graph_util.hpp"

namespace lidar {

class ObjectMatcher{
 public:
  ObjectMatcher() {}
  ~ObjectMatcher() {}

  // @brief set match distance maximum for matcher
  // @param[IN] match_distance_maximum: match distance maximum
  // @return true if set successfully, otherwise return false
  static bool SetMatchDistanceMaximum(const float match_distance_maximum);

  // @brief match detected objects to tracks
  // @param[IN] det_objects: point cloud network detected objects for matching
  // @param[IN] cls_objects: clustering objects for matching
  // @param[OUT] assignments: assignment pair of object & track
  // @param[OUT] unassigned_tracks: tracks without matched object
  // @param[OUT] unassigned_objects: objects without matched track
  // @return nothing
  void Match(const std::vector<lidar::ObjectPtr> &det_objects,
             const std::vector<lidar::ObjectPtr> &cls_objects);

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

  // @brief publish ros markers to show the association of objects
  // @param[IN] pub: ros publisher
  // @param[IN] header: ros msg header
  // @return nothing
  void PublishAssociations(const ros::Publisher &pub,
                           const std_msgs::Header &header);

  std::string Name() const { return "ObjectMatcher"; }

 protected:
  // @brief compute association matrix
  // @param[IN] tracks: maintained tracks for matching
  // @param[IN] tracks_predict: predicted states of maintained tracks
  // @param[IN] new_objects: recently detected objects
  // @param[OUT] association_mat: matrix of association distance
  // @return nothing
  void ComputeAssociateMatrix(
      const std::vector<lidar::ObjectPtr> &cls_objects,
      const std::vector<lidar::ObjectPtr> &det_objects,
      Eigen::MatrixXf* association_mat);

  // @brief compute connected components within given threshold
  // @param[IN] association_mat: matrix of association distance
  // @param[IN] connected_threshold: threshold of connected components
  // @param[OUT] track_components: connected objects of given tracks
  // @param[OUT] obj_components: connected tracks of given objects
  // @return nothing
  void ComputeConnectedComponents(
      const Eigen::MatrixXf& association_mat, const float connected_threshold,
      std::vector<std::vector<int>>* cls_components,
      std::vector<std::vector<int>>* det_components);

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
  std::vector<std::pair<int, int>> assignments_;
  std::vector<int> unassigned_cls_;
  std::vector<int> unassigned_det_;
  std::vector<lidar::ObjectPtr> det_objects_;
  std::vector<lidar::ObjectPtr> cls_objects_;

};  // class ObjectMatcher

}  // namespace lidar
