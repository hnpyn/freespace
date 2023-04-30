#ifndef MAPTYPE_H
#define MAPTYPE_H

#include <math.h>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <limits>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar {

enum TRAFFIC_DIRECTION {STRAIGHT, LEFT, RIGHT, U, UNKNOWND};
enum LINE_TYPE {SOLID, BROKEN, UNKNOWNT};
enum LINE_COLOR {WHITE, YELLOW, UNKNOWNC} ;

using pointxyz = pcl::PointXYZ;
using pointCloudPtr = pcl::PointCloud<pointxyz>::Ptr;
using pointrgba = pcl::PointXYZRGBA;
using pointRGBACloudPtr = pcl::PointCloud<pointrgba>::Ptr;
//#define M_PI 3.14159265357


// ############ geojson解析格式 #############

struct point_t
{
  double lon;
  double lat;
  double att;
};

using coordinate_t = std::vector<point_t>;
using polygon_t = std::vector<coordinate_t>;

struct line_t
{
  line_t() {}
  coordinate_t coordinates;
  std::string lineType;
  std::string lineColor;
  bool Implicit;
};


struct properties_t
{
  properties_t() {}
  std::string typeName;
  int featureId;
};

struct geometry_t
{
  geometry_t() {}
  std::string typeName;
  coordinate_t coordinates;
};

struct geometry_lane_t
{
  geometry_lane_t() {}
  std::string typeName;
  polygon_t polygon;
};

struct restriction_t
{
  restriction_t() {}
  std::string trafficDirection;
  std::vector<std::string> allowedVehicleTypesList;
  int speedLimitValue;
};

struct properties_lane_t
{
  properties_lane_t() {}
  size_t id;
  std::vector<size_t> fromLaneIdsList;
  std::vector<size_t> toLaneIdsList;
  std::vector<size_t> leftLanesList;
  std::vector<size_t> rightLanesList;
  std::vector<size_t> leftOpposingLanesList;
  std::vector<size_t> rightOpposingLanesList;
  line_t leftBoundaryLine;
  line_t rightBoundaryLine;
  line_t centerLine;
  line_t leftNavigableSurfaceLine;
  line_t rightNavigableSurfaceLine;
  restriction_t restriction;
  // association
  std::vector<size_t> crosswalksList; //人形横道
  std::vector<size_t> stoplinesList; //停止线
  std::vector<size_t> trafficCalmingsList; //减速带
  std::vector<size_t>  roisList; //交通标线
};

struct properties_junction_t
{
  properties_junction_t() {}
  std::string featureType;
  size_t featureId;
};

struct boundary_box_t
{
  boundary_box_t() {}
  pointxyz leftDown;
  pointxyz rightUp;
};

struct feature_lane_t
{
  feature_lane_t() {}
  std::string typeName;
  geometry_lane_t geometry;
  properties_lane_t properties;
};

struct feature_junction_t
{
  feature_junction_t() {}
  properties_junction_t properties;
  geometry_lane_t geometry;
};

// 其他feature
struct feature_polygon_t
{
  feature_polygon_t() {}
  polygon_t polygon;
  size_t id;
};

struct feature_line_t
{
  feature_line_t() {}
  coordinate_t coordinate;
  size_t id;
};

struct feature_association_t
{
  feature_association_t() {}
  std::vector<feature_polygon_t> trafficBumpList;
  std::vector<feature_polygon_t> crosswalkList;
  std::vector<feature_line_t> stoplineList;
  std::vector<std::pair<feature_polygon_t, std::string>> paint;
};


// ############ 给查找模块的格式 #############
// 已经转换为xy
//  idmap
struct idMap
{
  idMap() {}
  size_t id;
  std::vector<idMap*> from;
  std::vector<idMap*> to;
  std::vector<idMap*> left;
  std::vector<idMap*> right;
  std::vector<idMap*> leftOpp;
  std::vector<idMap*> rightOpp;
};

struct searchRoadBoundary_t
{
  searchRoadBoundary_t() {
    polygons.reset(new pcl::PointCloud<pointxyz>());
    leftBoundary.reset(new pcl::PointCloud<pointrgba>());
    rightBoundary.reset(new pcl::PointCloud<pointrgba>());
    leftNaviBoundary.reset(new pcl::PointCloud<pointrgba>());
    rightNaviBoundary.reset(new pcl::PointCloud<pointrgba>());
  }
  pointCloudPtr polygons;
  TRAFFIC_DIRECTION turn;
  int velocityLimit;

  pointRGBACloudPtr leftBoundary;
  pointRGBACloudPtr rightBoundary;
  pointRGBACloudPtr leftNaviBoundary;
  pointRGBACloudPtr rightNaviBoundary;

  pointRGBACloudPtr center;
  size_t id;

  boundary_box_t boundaryBox;// 包围框

  std::vector<size_t> idLIst;
  // association
  std::vector<size_t> crosswalksList; //人形横道
  std::vector<size_t> stoplinesList; //停止线
  std::vector<size_t> trafficCalmingsList; //减速带
  std::vector<size_t> roisList; //交通标线
};

struct searchJunction_t
{
  searchJunction_t() {
    junction.reset(new pcl::PointCloud<pointxyz>());
  }
  pointCloudPtr junction;
  pointxyz center;
  size_t id;
};

struct searchCoordinate_t
{
  searchCoordinate_t() {
    cloud.reset(new pcl::PointCloud<pointrgba>);
  }
  pointRGBACloudPtr cloud;
  size_t id;
};

struct searchAssocoation_t
{
  searchAssocoation_t() {}
  std::vector<searchCoordinate_t> trafficBumps;
  std::vector<searchCoordinate_t> crosswalkList;
  std::vector<searchCoordinate_t> stoplineList;
  std::vector<std::pair<searchCoordinate_t, std::string>> paint;
};


// ######### 高精度地图模块输出格式 ##########
struct roadBoundary_t
{
  roadBoundary_t() {
    leftBoundary.reset(new pcl::PointCloud<pointrgba>());
    rightBoundary.reset(new pcl::PointCloud<pointrgba>());
    stoplines.reset(new pcl::PointCloud<pointrgba>());
    leftNaviBoundary.reset(new pcl::PointCloud<pointrgba>());
    rightNaviBoundary.reset(new pcl::PointCloud<pointrgba>());
  }
  pointRGBACloudPtr leftBoundary;
  pointRGBACloudPtr rightBoundary;
  pointRGBACloudPtr stoplines;
  pointRGBACloudPtr leftNaviBoundary;
  pointRGBACloudPtr rightNaviBoundary;
  std::vector<pointRGBACloudPtr> bumps;
  std::vector<pointRGBACloudPtr> crosswalks;
  std::vector<std::pair<pointRGBACloudPtr, std::string>> rois;//1 左右

  TRAFFIC_DIRECTION dir;
  int speed;
  pointxyz pose;
  boundary_box_t boundaryBox;
  size_t id;
};

struct HdmapStruct
{
  HdmapStruct() {}
  std::vector<roadBoundary_t> roadBoundary;
  std::vector<pointRGBACloudPtr> junction;
};

struct orientation
{
  orientation() {}
  float x;
  float y;
  float z;
  float w;
};

struct trafficSigns
{
  trafficSigns() {}
  pointxyz poseSpeed;
  int speed;
  pointxyz poseTurn;
  TRAFFIC_DIRECTION turn;

};




//};
}



#endif // TYPE_H
