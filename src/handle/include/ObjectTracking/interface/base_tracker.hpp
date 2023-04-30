#ifndef BASE_TRACKER_H
#define BASE_TRACKER_H

#include "eigen3/Eigen/Core"
#include <memory>
#include "common/types/object.hpp"

namespace lidar {

struct TrackerOptions {
  TrackerOptions() = default;
  explicit TrackerOptions(Eigen::Matrix4d *pose) : velodyne_trans(pose) {}
//Eigen::Matrix4d* velodyne_trans;
  std::shared_ptr<Eigen::Matrix4d> velodyne_trans;
//  HdmapStructPtr hdmap = nullptr;
//  HDMapInput *hdmap_input = NULL;
};
class baseTracker {
 public:
  baseTracker() {}
  virtual ~baseTracker() {}

  virtual bool Init() = 0;

  // @brief: tracking objects.
  // @param [in]: current frame object list.
  // @param [in]: timestamp.
  // @param [in]: options.
  // @param [out]: current tracked objects.
  virtual bool Track(const std::vector<std::shared_ptr<lidar::Object>> &objects,
                     double timestamp, const TrackerOptions &options,
                     std::vector<std::shared_ptr<lidar::Object>> &tracked_objects) = 0;

  virtual void predict_objects(std::vector<lidar::ObjectPtr> &predict_objects, const double curTime) = 0;
  virtual std::string name() const = 0;

};

}//namespace lidar

#endif // BASE_TRACKER_H
