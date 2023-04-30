#ifndef OBJECTS_TRACKED_H
#define OBJECTS_TRACKED_H

#include <memory>
#include <eigen3/Eigen/Core>

#include "common/types/object.hpp"
#include "common/geometry_util.hpp"

namespace lidar{

struct objects_tracked
{
  objects_tracked();
  explicit objects_tracked(std::shared_ptr<lidar::Object> objects_ptr) : objectsPtr(objects_ptr) {
    size = Eigen::Vector3f(float(objects_ptr->length), float(objects_ptr->width), float(objects_ptr->height));
    center = objects_ptr->ground_center.cast<float>();
    baryCenter = GetCloudBarycenter<lidar::PointI>(objectsPtr->cloud).cast<float>();
    direction = objects_ptr->direction.cast<float>();
    anchorCenter = baryCenter;
    velocity = Eigen::Vector3f::Zero();
    acceleration = Eigen::Vector3f::Zero();
    type = objects_ptr->type;
  }
  //obj ptr
  std::shared_ptr<lidar::Object> objectsPtr;
  //bary points
  Eigen::Vector3f baryCenter;
  //bbox
  Eigen::Vector3f size;
  Eigen::Vector3f center;
  Eigen::Vector3f direction;
  Eigen::Vector3f anchorCenter;
  //states
  Eigen::Vector3f velocity;
  Eigen::Vector3f acceleration;
  Eigen::Matrix3f velocity_uncertainty;
  //计算距离得到关联得分
  float association_score = 0.0f;
  //type
  lidar::ObjectType type;
  void clone(const objects_tracked& rhs) {
    *this = rhs;
    objectsPtr.reset(new lidar::Object());
    objectsPtr->clone(*rhs.objectsPtr);
  }
};
}//namespace lidar
#endif // OBJECTS_TRACKED_H
