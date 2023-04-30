#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <memory>
#include "common/types/object.hpp"
#include "objects_tracked.hpp"
#include "ObjectTracking/interface/base_filter.hpp"
#include "ObjectTracking/kalman_filter.hpp"

namespace lidar {

class trajectory
{
private:
  void SmoothTrackVelocity(const std::shared_ptr<objects_tracked>& new_object, const double time_diff);
  void SmoothTrackOrientation();
  double VectorTheta2dXy(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2);
  bool CheckTrackStaticHypothesisByVelocityAngleChange(const std::shared_ptr<lidar::Object>& new_object, const double time_diff);
  bool CheckTrackStaticHypothesis(const std::shared_ptr<lidar::Object>& new_object, const double time_diff);
  void ComputeBboxSizeCenter(lidar::PointICloudPtr cloud,
                             const Eigen::Vector3d& direction,
                             Eigen::Vector3d* size, Eigen::Vector3d* center);
  double Clamp(const double value, double bound1, double bound2);
public:
  explicit trajectory(std::shared_ptr<objects_tracked> obj);
  Eigen::VectorXf Predict(const double time_diff);
  void update_with_object(std::shared_ptr<objects_tracked> &new_object,
                          const double time_diff);
  void update_without_object(const double time_diff) ;
  void update_without_object(const Eigen::VectorXf& predict_state,
                             const double time_diff);

  int GetNextTrackId();
  ~trajectory();



public:
  int idx_;
  int age_;
  double period_;
  int total_visible_count_;

  std::shared_ptr<objects_tracked> current_object_;

  //未关联到的帧数
  int consecutive_invisible_count_;

  baseFilter* filter_;

  //states
  bool is_static_hypothesis_;
  Eigen::Vector3f belief_anchor_point_;
  Eigen::Vector3f belief_velocity_;
  Eigen::Matrix3f belief_velocity_uncertainty_;
  Eigen::Vector3f belief_velocity_accelaration_;

  //轨迹初始化参数
  static int s_track_idx_;//每条轨迹的ID
  static int s_track_cached_history_size_maximum_;
  static double s_speed_noise_maximum_;
  static double s_acceleration_noise_maximum_;

  // history
  std::deque<std::shared_ptr<objects_tracked>> history_objects_;
};

typedef trajectory* trajectoryPtr;

}//namespace lidar


#endif // TRAJECTORY_H
