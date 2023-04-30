#ifndef TRACK_DATA_H
#define TRACK_DATA_H

#include "trajectory.hpp"

namespace lidar {

class track_data
{
public:
  track_data();

  void add_track(trajectoryPtr trajectorys){ traj_.push_back(trajectorys);}
  std::vector<trajectoryPtr> &get_track(){return traj_;}
  void clear(){std::vector<trajectoryPtr>().swap(traj_);}
  int Size(){return traj_.size();}
  int delete_lost_traj();


private:
  std::vector<trajectoryPtr> traj_;
  const static int s_track_consecutive_invisible_maximum_;
  const static float s_track_visible_ratio_minimum_;

};



}//namespace lidar


#endif // TRACK_DATA_H
