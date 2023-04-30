#ifndef HUNG_KAL_TRACKER_H
#define HUNG_KAL_TRACKER_H

#include "ObjectTracking/interface/base_tracker.hpp"
#include "common/types/object.hpp"
#include "common/types/objects_tracked.hpp"
#include "common/types/track_data.hpp"
#include "ObjectTracking/interface/base_matcher.hpp"
#include "hungarian_matcher.hpp"

#include <numeric>

namespace lidar {

class hung_kal_tracker: public baseTracker
{
public:
  hung_kal_tracker();
  bool track_first_frame(const std::vector<std::shared_ptr<lidar::Object>> &objects, double timestamps,
                         std::vector<std::shared_ptr<lidar::Object> > &track_objects);
  void construct_objects_tracked(const std::vector<std::shared_ptr<lidar::Object>> &objects,
                                 std::vector<std::shared_ptr<objects_tracked> > &objs_transform);
  void get_transform_matrix(const TrackerOptions &options);
  void transform_objects(std::shared_ptr<objects_tracked> &obj_transform);

  void creat_trajectory(const std::vector<std::shared_ptr<objects_tracked>> &obj_transform,
                        std::vector<int> obj_unassigned);
  void get_objects(std::vector<std::shared_ptr<lidar::Object> > &tracked_objs);
  void predict_tracks(std::vector<Eigen::VectorXf> &predictStates, const double timeDiff);
  void predict_objects(std::vector<lidar::ObjectPtr> &predict_objects, const double curTime);

  void update_assigned_tracks(std::vector<Eigen::VectorXf> &predictStates,
                              std::vector<std::shared_ptr<objects_tracked>> &obj_transform,
                             const std::vector<std::pair<int, int>> assignments, const double timeDiff);
  void update_unassigned_tracks(std::vector<Eigen::VectorXf> &predictStates,
                               std::vector<int> unassignments,
                               const double timeDiff);
  void delete_tracks();

  bool Init();
  bool Track(const std::vector<std::shared_ptr<lidar::Object> > &objects, double timestamps,
       const TrackerOptions &options, std::vector<std::shared_ptr<lidar::Object> > &tracked_objects);
  std::string name() const { return "hungarianKalmanTracker"; }

private:
bool first_frame_ = true;
double timestamps_ = 0;
Eigen::Vector3d transform_global2local;
Eigen::Matrix3d transform_local;

std::unique_ptr<baseMatcher> p_base_match_;
public:
track_data trajectory_data_;

};

}//namespace lidar

#endif // HUNG_KAL_TRACKER_H
