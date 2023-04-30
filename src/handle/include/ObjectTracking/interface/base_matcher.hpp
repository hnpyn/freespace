#ifndef BASE_MATCHER_H
#define BASE_MATCHER_H

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "common/types/objects_tracked.hpp"
#include "common/types/trajectory.hpp"

namespace lidar {

class baseMatcher {
 public:
  baseMatcher() {}
  virtual ~baseMatcher() {}

  // @brief match detected objects to maintained tracks
  // @param[IN] objects: detected objects
  // @param[IN] tracks: maintained tracks
  // @param[IN] tracks_predict: predicted states of maintained tracks
  // @param[OUT] assignments: matched pair of <track, object>
  // @param[OUT] unassigned_tracks: unmatched tracks
  // @param[OUT] unassigned_objects: unmatched objects
  // @return nothing
  virtual void Match(std::vector<std::shared_ptr<objects_tracked>>* objects,
                     const std::vector<trajectoryPtr>& tracks,
                     const std::vector<Eigen::VectorXf>& tracks_predict,
                     std::vector<std::pair<int, int>>* assignments,
                     std::vector<int>* unassigned_tracks,
                     std::vector<int>* unassigned_objects) = 0;

  // @brief get name of matcher
  // @return name of matcher
  virtual std::string Name() const = 0;


};  // class BaseObjectTrackMatcher
}//namespace lidar
#endif // BASE_MATCHER_H
