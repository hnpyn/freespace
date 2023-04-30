#include <iostream>
#include <fstream>
#include <cmath>
#include <eigen3/Eigen/Core>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include "LocalMap/local_map_filter.h"

const double PI = 3.1415926;

namespace lidar {

LocalMapFilter::LocalMapFilter()
{
}

void LocalMapFilter::LoadLocalMap(const std::string &file_name)
{
  std::ifstream mapFile;
  mapFile.open(file_name.c_str());
  if (!mapFile)
    std::cerr << "Can not open local map file!" << std::endl;

  char comma;
  size_t szLane;
  size_t laneId, szPoints;
  size_t globalIdx;
  lidar::PointD point;
  lidar::PolygonDType polygon;
  mapFile >> szLane;
  std::cout << "szLane: " << szLane << std::endl;
  for (size_t i = 0; i < szLane; ++i) {
    mapFile >> laneId >> szPoints;
    std::cout << laneId << " " << szPoints << std::endl;
    for (size_t j = 0; j < szPoints; ++j) {
      mapFile >> globalIdx >> comma >> point.x >> comma >> point.y >> comma >> point.z;
      polygon.points.push_back(point);
    }
    polygons.push_back(polygon);
    polygon.points.clear();
  }
}

static void getMinMax3D(const lidar::PointDCloud &cloud,
                        lidar::PointD &min_pt, lidar::PointD &max_pt)
{
  if (cloud.size() <= 0)
    return;
  min_pt = cloud.points.at(0);
  max_pt = min_pt;
  for (size_t i = 0; i < cloud.points.size(); ++i) {
    if (cloud.points[i].x < min_pt.x)
      min_pt.x = cloud.points[i].x;
    if (cloud.points[i].x > max_pt.x)
      max_pt.x = cloud.points[i].x;
    if (cloud.points[i].y < min_pt.y)
      min_pt.y = cloud.points[i].y;
    if (cloud.points[i].y > max_pt.y)
      max_pt.y = cloud.points[i].y;
    if (cloud.points[i].z < min_pt.z)
      min_pt.z = cloud.points[i].z;
    if (cloud.points[i].z > max_pt.z)
      max_pt.z = cloud.points[i].z;
  }
}

void LocalMapFilter::SearchPolygons(const lidar::Point &pos)
{
  selectedPolygons.clear();
  selectedOnePolygons.clear();
  size_t szPolygon = polygons.size();
  lidar::PointD minPt, maxPt;
  for (size_t i = 0; i < szPolygon; ++i) {
    getMinMax3D(polygons.at(i), minPt, maxPt);
    if (pos.x < minPt.x || pos.x > maxPt.x ||
        pos.y < minPt.y || pos.y > maxPt.y) {
        std::cout << "DEBUG:? " << pos.x << "||" << pos.y << std::endl;
        std::cout << "DEBUG:? " << minPt.x << "||" << minPt.y << std::endl;
        std::cout << "DEBUG:? " << maxPt.x << "||" << maxPt.y << std::endl;
      continue;
    }
    std::cout << "DEBUG:? " << i << std::endl;
    size_t j, k;
    auto vertexs = polygons.at(i).points;
    size_t szVertex = vertexs.size();
    bool bInsidePoly = false;
    for (j = 0, k = szVertex - 1; j < szVertex; k = j++) {
      if ( ((vertexs.at(j).y > pos.y) != (vertexs.at(k).y > pos.y)) &&
           (pos.x < (vertexs.at(k).x - vertexs.at(j).x)*(pos.y - vertexs.at(j).y)/
            (vertexs.at(k).y - vertexs.at(j).y) + vertexs.at(j).x) ) {
        bInsidePoly = !bInsidePoly;
      }
    }
    if (bInsidePoly) {
      if (i > 0)
        selectedPolygons.push_back(polygons.at(i-1));
      selectedPolygons.push_back(polygons.at(i));
      selectedOnePolygons.push_back(polygons.at(i));
      if (i < szPolygon - 1)
        selectedPolygons.push_back(polygons.at(i+1));
      break;
    }
  }
  std::cout << "selectedPolygons: " << selectedPolygons.size() << std::endl;
}

void LocalMapFilter::SetTransform(const lidar::Point &location,
                                  const Eigen::Quaterniond &quaternion)
{
  Eigen::Matrix3d rotation = quaternion.toRotationMatrix();
  transform.block<3, 3>(0, 0) = rotation.inverse();
  transform.block<3, 1>(0, 3) = Eigen::Vector3d(location.x, location.y, location.z);
}

void LocalMapFilter::ExtractFilterCloud(lidar::PointICloudPtr &raw_cloud,
                                        lidar::PointICloudPtr &filter_cloud,
                                        lidar::PointIndices &indices)
{
  filter_cloud->clear();
  for (size_t i = 0; i < indices.indices.size(); ++i) {
    int idx = indices.indices.at(i);
    filter_cloud->points.push_back(raw_cloud->points[idx]);
  }
  filter_cloud->header = raw_cloud->header;
}

void LocalMapFilter::Filter(lidar::PointICloudPtr &cloud,
                            lidar::PointICloudPtr &bg_cloud,
                            const lidar::Point &location,
                            const Eigen::Quaterniond &quaternion)
{
  SearchPolygons(location);
  SetTransform(location, quaternion);

  if (selectedPolygons.size() == 0) {
    std::cerr << "No local map found!" << std::endl;
    return;
  }

  // lidar::HdmapROIFilter roiFilter;
  lidar::PointIndices roi_indices, bg_indices;
  // roiFilter.Filter(cloud, selectedPolygons, transform, roi_indices, bg_indices);
  lidar::PointICloudPtr roi_cloud(new lidar::PointICloud);
  ExtractFilterCloud(cloud, roi_cloud, roi_indices);
  ExtractFilterCloud(cloud, bg_cloud, bg_indices);
  cloud = roi_cloud;
}

static void transformPointCloud(lidar::PointDCloud &cloud_in, 
                       lidar::PointDCloud &cloud_out, Eigen::Matrix4d &transform)
{
  Eigen::Vector3d translation = transform.block<3, 1>(0, 3);
  Eigen::Matrix3d rotation = transform.block<3, 3>(0, 0);
  for (size_t i = 0; i < cloud_in.points.size(); ++i) {
    Eigen::Vector3d point_in(cloud_in.points[i].x,
                             cloud_in.points[i].y,
                             cloud_in.points[i].z);
    Eigen::Vector3d point_out;
    point_out = rotation * (point_in - translation);
    lidar::PointD tmp_point;
    tmp_point.x = point_out(0);
    tmp_point.y = point_out(1);
    tmp_point.z = point_out(2);
    cloud_out.points.push_back(tmp_point);
  }
}

void LocalMapFilter::CalculateLine(lidar::RoiPoint &roi_point)
{
    lidar::PointDCloud transPolygon;
    lidar::roipoint param;
    transformPointCloud(selectedOnePolygons.at(0), transPolygon, transform);
    
    for (size_t j = 0; j < transPolygon.size(); ++j) 
    {
      // 计算roi点与原点的距离
      param.dis = sqrt(transPolygon.points.at(j).x * transPolygon.points.at(j).x 
                                          + transPolygon.points.at(j).y * transPolygon.points.at(j).y);
      // 计算roi点与原点的连线的夹角
      if (transPolygon.points.at(j).y >= 0)
        param.ag = atan2(transPolygon.points.at(j).y, transPolygon.points.at(j).x) * 180 / PI;
      else
        param.ag = atan2(transPolygon.points.at(j).y, transPolygon.points.at(j).x) * 180 / PI + 360;
       // 存储roi点坐标
      param.x = transPolygon.points.at(j).x;
      param.y = transPolygon.points.at(j).y;
      param.z = 0.0;
      roi_point.push_back(param);
    }
// std::cout <<"size: " << roi_point.size() << "!!!!!!!!!!!!!" << std::endl;
// for (int n = 0; n < roi_point.size(); n++)
// {
//   if (120 <=roi_point.at(n).ag && roi_point.at(n).ag <= 240)
//   {std::cout << roi_point.at(n).ag << "||" << roi_point.at(n).dis << std::endl;}
// }
//   double res;
//   res = atan2(-1, 0) * 180 / PI;
//   std::cout << "lalalalala " << res << std::endl;
}

void LocalMapFilter::PublishPolygons(const ros::Publisher &pub, const std_msgs::Header &header)
{
  //clear all markers before
  visualization_msgs::MarkerArray empty_markers;
  visualization_msgs::Marker clear_marker;
  clear_marker.header = header;
  clear_marker.ns = "objects";
  clear_marker.id = 0;
  clear_marker.action = visualization_msgs::Marker::DELETEALL;
  clear_marker.lifetime = ros::Duration();
  empty_markers.markers.push_back(clear_marker);
  pub.publish(empty_markers);

  visualization_msgs::MarkerArray polygon_markers;
  
  for (size_t i = 0u; i < selectedPolygons.size(); ++i) {
      visualization_msgs::Marker polygon;
      polygon.header = header;
      polygon.ns = "polygons";
      polygon.id = i;
      polygon.type = visualization_msgs::Marker::LINE_LIST;
      
      lidar::PointDCloud transPolygon;
      geometry_msgs::Point point, firstPoint;
      // lidar::linepa param;
      transformPointCloud(selectedPolygons.at(i), transPolygon, transform);
      
      for (size_t j = 0; j < transPolygon.size(); ++j) {
        
        point.x = transPolygon.points.at(j).x;
        point.y = transPolygon.points.at(j).y;
        point.z = transPolygon.points.at(j).z;
        point.z = 0.0;
        
        polygon.points.push_back(point);
        
        if (j > 0)
          polygon.points.push_back(point);
        else
          firstPoint = point;
      }
      polygon.points.push_back(firstPoint);

      polygon.scale.x = 0.1;
      std_msgs::ColorRGBA color;
      color.a = 1.0; color.r = 0.0; color.g = 1.0; color.b = 0.0;
      polygon.color = color;
      polygon_markers.markers.push_back(polygon);
  }

  pub.publish(polygon_markers);
}

} //namespace lidar
