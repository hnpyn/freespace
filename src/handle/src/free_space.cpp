#include "free_space.h"
#include <stdio.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <ctime>
// // 声明全局变量用于存储当前帧和上一帧的distance
// extern double temp;
// 初始化一个61个值为1的数组
std::vector<double> points_distance_1(601, 1.0);
// extern std::vector<double> points_distance_2;
// extern std::vector<double> points_distance_3;
// extern std::vector<double> points_distance_4;
// extern std::vector<double> points_distance_5;
// // 使用队列存储各个帧的distance
// extern std::queue<double> point_distance;

// 初始化全局变量
// double temp = 1.0;

namespace lidarPerception {


FreeSpace::FreeSpace()
{
}

void FreeSpace::get_free_space(const std_msgs::Header header,const PointICloudPtr &point_cloud,
                               std::vector<int> &list, lidar:: RoiPoint roi_point,
                               PointICloudPtr out_point_cloud,
                               PointICloudPtr out_point_cloud_xy,
                               visualization_msgs::MarkerArray &free_space_marker)
{ 
  std::clock_t startTime, endTime;
  int polar_grid_nums_ = std::ceil((config_.angle_end - config_.angle_begin) / config_.angle_resolution);

  //建立极坐标栅格
  startTime = clock();
  int gridRows = std::ceil((config_.distance_max - config_.distance_min) / config_.distance_resolution);
  int gridCols = polar_grid_nums_;
  endTime = clock();
  std::cout << "建立极坐标栅格 - time: " 
                     << (double) (endTime - startTime) / CLOCKS_PER_SEC << " s" << std::endl;

  //极坐标的freespace
  startTime = clock();
  std::vector<std::vector<grid_feature> > polar_grid_points(gridRows, std::vector<grid_feature> (gridCols));//最大高度 最小高度 最小距离 点个数
  construct_polar_grid(point_cloud, polar_grid_points);
  endTime = clock();
  std::cout << "极坐标free space - time: " 
                     << (double) (endTime - startTime) / CLOCKS_PER_SEC << " s" << std::endl;

  //判定栅格属性
  startTime = clock();
  std::vector<std::vector<int> > objects_grid(gridRows, std::vector<int> (gridCols, 0));
  check_grid(polar_grid_points, gridRows, gridCols, objects_grid);
  endTime = clock();
  std::cout << "判定栅格属性 - time: " 
                     << (double) (endTime - startTime) / CLOCKS_PER_SEC << " s" << std::endl;

  //找到最近距离
  startTime = clock();
  std::vector<double> point_list;
  point_list.resize(polar_grid_nums_);
  for (int i = 0; i < polar_grid_nums_; i++)
  {
    point_list.at(i) = config_.distance_max;
  }

//  get_closest_distance(polar_grid_points, objects_grid,
//                       gridRows, gridCols, point_list);
  get_closest_distance_2(polar_grid_points, objects_grid,
                           gridRows, gridCols, point_list);
  endTime = clock();
  std::cout << "找到最近距离 - time: " 
                     << (double) (endTime - startTime) / CLOCKS_PER_SEC << " s" << std::endl;

  //修正free_space点
  startTime = clock();
  correct_points(point_list);
  endTime = clock();
  std::cout << "修正free space点 - time: " 
                     << (double) (endTime - startTime) / CLOCKS_PER_SEC << " s" << std::endl;

  //修正free_space角度
  startTime = clock();
  correct_points_angle(point_list);
  endTime = clock();
  std::cout << "修正free space角度 - time: " 
                     << (double) (endTime - startTime) / CLOCKS_PER_SEC << " s" << std::endl;

  //删掉空出的角度
  startTime = clock();
  std::vector<double> out_point_list;
  shrink_angle(point_list, out_point_list);
  endTime = clock();
  std::cout << "删除空出角度 - time: " 
                     << (double) (endTime - startTime) / CLOCKS_PER_SEC << " s" << std::endl;

  //输出xy坐标
  startTime = clock();
  std::vector<PointI> out_point_list_xy;
  get_point_list_xy(out_point_list, out_point_list_xy, out_point_cloud_xy);
  endTime = clock();
  std::cout << "输出xy坐标 - time: " 
                     << (double) (endTime - startTime) / CLOCKS_PER_SEC << " s" << std::endl;

  //转换坐标及输出格式
  startTime = clock();
  transform_formats(out_point_list_xy, list);
  endTime = clock();
  std::cout << "转换坐标及输出格式 - time: " 
                     << (double) (endTime - startTime) / CLOCKS_PER_SEC << " s" << std::endl;

  //freespace 三维点坐标
  startTime = clock();
  draw_points(point_list, out_point_cloud);
  endTime = clock();
  std::cout << "建立free space三维点坐标 - time: " 
                     << (double) (endTime - startTime) / CLOCKS_PER_SEC << " s" << std::endl;

  // roi修正freespace
  startTime = clock();
  PointICloudPtr temp_point_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  correct_freespace(roi_point, out_point_cloud, temp_point_cloud);
  endTime = clock();
  std::cout << "roi修正free space - time: " 
                     << (double) (endTime - startTime) / CLOCKS_PER_SEC << " s" << std::endl;

  //draw
  startTime = clock();
  draw_lines(header, out_point_cloud, free_space_marker);
  endTime = clock();
  std::cout << "画出free space - time: " 
                     << (double) (endTime - startTime) / CLOCKS_PER_SEC << " s" << std::endl;

}

void FreeSpace::transform_formats(std::vector<PointI> &out_point_list_xy, std::vector<int> &list)
{
  for (int i = 0; i < out_point_list_xy.size(); i++)
  {
    double xx = (out_point_list_xy[i].x + 30.0) * 10.0;
    int x = 0;
    if (xx > 0)
      x = std::floor(xx);
    else
      x = std::ceil(xx);
    list.push_back(x);

    double yy = ((out_point_list_xy[i].y + 30.0) * 10.0) * (-1.0);
    int y = 0;
    if (yy > 0)
      y = std::floor(yy);
    else
      y = std::ceil(yy);
    list.push_back(y);
  }
}

void FreeSpace::get_point_list_xy(std::vector<double> &out_point_list,
                                  std::vector<PointI> &out_point_list_xy,
                                  PointICloudPtr out_point_cloud_xy)
{
  int listSize = out_point_list.size();
  int halfListSize = listSize / 2;
  for (int i = 0; i < listSize; i++)
  {
    double theta;
        if (i < halfListSize)
          theta = get_theta(i) + config_.t_angle_begin + config_.angle_resolution / 2;
        else
          theta = get_theta(i) + config_.t_angle_begin + config_.angle_resolution / 2 + config_.t_angle_free * 2;
    double distance = out_point_list[i];
    PointI outPoint;
    get_point_position(theta, distance, outPoint);
    out_point_list_xy.push_back(outPoint);
    out_point_cloud_xy->points.push_back(outPoint);
  }
}

void FreeSpace::shrink_angle(std::vector<double> &point_list, std::vector<double> &out_point_list)
{
  double remain_angle = 90.0 - config_.t_angle_free - (config_.t_angle_begin - 90.0);
  int remain_points = std::ceil(remain_angle / config_.angle_resolution);
  int size = point_list.size();
  for (int i = 0; i < size; i++)
  {
    if (i < remain_points || i >= size - remain_points)
      out_point_list.push_back(point_list.at(i));
  }
}

void FreeSpace::correct_points_angle(std::vector<double> &point_list)
{
  double former_angle = std::ceil((config_.t_angle_begin - config_.angle_begin) / config_.angle_resolution);
  double back_angle = std::ceil((config_.t_angle_end) / config_.angle_resolution);
  std::vector<double> tmp_list = point_list;

  point_list.clear();
  for (double i = 0; i < tmp_list.size(); i++)
  {
    if (i >= former_angle && i < back_angle)
    {
      point_list.push_back(tmp_list[i]);
    }
  }
}

void FreeSpace::correct_points(std::vector<double> &point_list)
{
  std::vector<int> points_has_corrected;
  int pointsSize = point_list.size();
  int nearstPointsNums = 40;
  for (int i = 0; i < nearstPointsNums; i++)
  {
    point_list.push_back(point_list[i]);
  }

  bool flags = true;
  for (int i = 0; i < (pointsSize - nearstPointsNums); i++)
  {
    flags = check_corrected_points(i, points_has_corrected);
    if (!flags)
      continue;

    double curTheta = get_theta(i) + config_.angle_begin;
    double curDistance = point_list[i];
    PointI curPoint;
    get_point_position(curTheta, curDistance, curPoint);
    //后向
    for (int j = nearstPointsNums; j > 0; j--)
    {
      double theta = get_theta(i + j) + config_.angle_begin;
      double distance = point_list[i + j];
      PointI laterPoint;
      get_point_position(theta, distance, laterPoint);
      double dis = calculate_points_dis(laterPoint, curPoint);
      if (dis < config_.t_correct_dis)
      {
        for (int k = 0; k < j; k++)
        {
          double correctX = (laterPoint.x - curPoint.x) / j * k + curPoint.x;
          double correctY = (laterPoint.y - curPoint.y) / j * k + curPoint.y;
          double correctedDis = sqrt(correctX * correctX + correctY * correctY);
          if (correctedDis < point_list[i + k])
          {
            point_list[i + k] = correctedDis;
            points_has_corrected.push_back(i + k);
          }
        }
      }
    }
  }
  for (int i = 0; i < nearstPointsNums; i++)
  {
    point_list.pop_back();
  }
}

bool FreeSpace::check_corrected_points(int i, std::vector<int> points_has_corrected)
{
  for (int ii = 0; ii < points_has_corrected.size(); ii++)
  {
    if (i == points_has_corrected[ii])
      return false;
  }
  return true;
}

double FreeSpace::calculate_points_dis(const PointI pa, const PointI pb)
{
  double dis = sqrt((pa.x - pb.x) * (pa.x - pb.x) + (pa.y - pb.y) * (pa.y - pb.y));
  return dis;
}

double FreeSpace::calculate_dis(const PointI p)
{
  double dis = sqrt(p.x  * p.x + p.y * p.y);
  return dis;
}

void FreeSpace::correct_freespace(lidar::RoiPoint roi_point, 
          PointICloudPtr out_point_cloud, PointICloudPtr &temp_point_cloud)
{
  int listSize = out_point_cloud->points.size();
  int roiSize = roi_point.size();
  lidar::roipoint tempPoint;
  lidar::RoiQueue twoPoints;
  PointI outPoint, oPoint;
  // temp_point_cloud->points.clear();
  oPoint.x = 0.0;
  oPoint.y = 0.0;
  oPoint.z = 0.0;
  oPoint.intensity = 255.0;
  temp_point_cloud->points.push_back(oPoint);
  int start_num = 0;
  int correctPointsNums;
  int t_correctNum = 0;
  for (int i = 0; i < roiSize; i++)
  {
      if (roi_point.at(i).ag >= 60 && roi_point.at(i).ag <= 300)
      {
        tempPoint.x = roi_point.at(i).x;
        tempPoint.y = roi_point.at(i).y;
        tempPoint.ag = roi_point.at(i).ag;
        tempPoint.dis = roi_point.at(i).dis;
        twoPoints.push(tempPoint);
      }
      if (twoPoints.size() > 1)
      {
        lidar::roipoint firstPoint = twoPoints.front();
        twoPoints.pop();
        lidar::roipoint secondPoint = twoPoints.front();
        correctPointsNums = 0;
        for (int j = start_num; j < listSize; j++)
        {
          double curTheta = get_theta(j) + config_.t_angle_begin;
          // std::cout << "firstPoint: " << firstPoint.ag << " secondPoint: " << secondPoint.ag << std::endl;
          if (curTheta > firstPoint.ag)
          {
            if (curTheta <= secondPoint.ag)
            {
              start_num = j;
              correctPointsNums += 1;
            }
            else break;
          }
        }
        // std::cout << "start_num: " << start_num << std::endl;
        // std::cout << "correctPointsNums: " << correctPointsNums << std::endl;
        // for (int m = start_num; m < (start_num + correctPointsNums); m++)
        // {
          if (start_num >= listSize - 1) break;
          int m = start_num;
          int n = correctPointsNums;
            for (int k = 1; k < n; k ++)
            {
              double correctX = (secondPoint.x - firstPoint.x) / n * k + firstPoint.x;
              double correctY = (secondPoint.y - firstPoint.y) / n * k + firstPoint.y;
              double correctedDis = sqrt(correctX * correctX + correctY * correctY);
              double theta = get_theta(m + k - 1) + config_.t_angle_begin;
              double distance;
              // std::cout << "debug: m+k-1 -- " << m+k-1 << std::endl;
              double freeSpaceDistance = calculate_dis(out_point_cloud->points.at(m+k-1));
              // std::cout << "debug: ..." << std::endl;
              if (correctedDis < freeSpaceDistance){distance = correctedDis;}
              else{distance = freeSpaceDistance;}
              get_point_position(theta, distance, outPoint);
              temp_point_cloud->points.push_back(outPoint);
              //point_list_temp.push_back(distance);
            }
          t_correctNum += correctPointsNums;
        // }
      }
  }
  temp_point_cloud->points.push_back(oPoint);
  // std::cout << "debug: " 
  //                    << "\n" << "points for draw: " << temp_point_cloud->points.size()  
  //                    << "\n" << "total correct nums: " << t_correctNum << std::endl;
}

#include <pcl/filters/voxel_grid.h>
void FreeSpace::draw_lines(const std_msgs::Header header, PointICloudPtr out_point_cloud,
                           visualization_msgs::MarkerArray &free_space_marker)
{
  int pointsSize = out_point_cloud->points.size();

  // // 添加一个原点
  // PointI oPoint;
  // oPoint.x = 0.0;
  // oPoint.y = 0.0;
  // oPoint.z = 0.0;
  // oPoint.intensity = 255.0;
  // out_point_cloud->points.push_back(oPoint);

  for (int i = 0; i < pointsSize; i++)
  {
    visualization_msgs::Marker lines;
    lines.header = header;
    lines.ns = "lines";
    lines.id = i;
    lines.action = visualization_msgs::Marker::ADD;
    lines.lifetime = ros::Duration(0.145);
    lines.type = visualization_msgs::Marker::LINE_LIST;
    geometry_msgs::Point position1, position2;

    position1.x = out_point_cloud->points[i].x;
    position1.y = out_point_cloud->points[i].y;
    position1.z = out_point_cloud->points[i].z;
    if (i == pointsSize - 1)
    {
      if ((config_.t_angle_end - config_.t_angle_begin) == 360.0)
      {
        position2.x = out_point_cloud->points[0].x;
        position2.y = out_point_cloud->points[0].y;
        position2.z = out_point_cloud->points[0].z;
      }
    }
    else
    {
      position2.x = out_point_cloud->points[i + 1].x;
      position2.y = out_point_cloud->points[i + 1].y;
      position2.z = out_point_cloud->points[i + 1].z;
    }
    lines.points.push_back(position1);
    lines.points.push_back(position2);
    lines.scale.x = 0.1;
    lines.color.r = 1.0;
    lines.color.a = 1.0;
    free_space_marker.markers.push_back(lines);
  }

  //非360度画原点与每一个点的连线
  if ((config_.t_angle_end - config_.t_angle_begin) < 360)
  {
    visualization_msgs::Marker lines;
    lines.header = header;
    lines.ns = "lines";
    lines.id = pointsSize;
    lines.action = visualization_msgs::Marker::ADD;
    lines.lifetime = ros::Duration(0.145);
    lines.type = visualization_msgs::Marker::LINE_LIST;
    geometry_msgs::Point position1, position2;

    for (int i = 0; i < pointsSize; i++ )
    {
      position1.x = out_point_cloud->points[i].x;
      position1.y = out_point_cloud->points[i].y;
      position1.z = out_point_cloud->points[i].z;
      position2.x = 0;
      position2.y = 0;
      position2.z = 0;
      lines.points.push_back(position1);
      lines.points.push_back(position2);
    }
    lines.scale.x = 0.1;
    lines.color.g= 1.0;
    lines.color.a = 1.0;
    free_space_marker.markers.push_back(lines);
  }
}

void FreeSpace::filter_points_cloud(const PointICloudPtr point_cloud, PointICloudPtr filter_point_cloud)
{
  pcl::PassThrough<pcl::PointXYZI> pass1;
  //pcl::PassThrough<pcl::PCLPointCloud2> pass;
  pass1.setInputCloud (point_cloud);
  pass1.setFilterFieldName ("z");
  pass1.setFilterLimits (-5.0, -1.0);
  pass1.setFilterLimitsNegative (false);
  pass1.filter (*filter_point_cloud);
}

//转换到点云坐标
void FreeSpace::draw_points(const std::vector<double> &point_list, PointICloudPtr &out_point_cloud)
{
  int listSize = point_list.size();
  int halfListSize = listSize / 2;
  for (int i = 0; i < listSize; i++)
  {
    double theta;
//    if (i < halfListSize)
//      theta = get_theta(i) + t_angle_begin_ + angle_resolution_ / 2;
//    else
//      theta = get_theta(i) + t_angle_begin_ + angle_resolution_ / 2 + t_angle_free * 2;
    theta = get_theta(i) + config_.t_angle_begin + config_.angle_resolution / 2;
    double distance = point_list[i];

    // 滤除噪点  
    if (i > 0 && point_list[i] - point_list[i-1] < -3.5 
                  && point_list[i] - point_list[i+1] < -3.5)
    {
      // distance = (point_list[i-1] + point_list[i]) / 2;
      distance = std::min(point_list[i-1], point_list[i+1]) ;
    }
    else if (i > 0 && point_list[i] - point_list[i-1] > 3.5
                            && point_list[i] - point_list[i+1] > 3.5)
    {
      // distance = (point_list[i-1] + point_list[i]) / 2;
      distance = std::min(point_list[i-1], point_list[i+1]) ;
    }
    
    // // 上一帧的distanc
    // double distance_;
    // distance_ = temp;
    // // 计算当前帧distance与前一帧distance的差值
    // if (distance - distance_ > 0)
    // {
    //   if (distance - distance_ >=15)
    //   {
    //     distance = (1 + distance_ / (10 * (distance - distance_))) * distance_ + 1;
    //   }
    //   else
    //   {
    //     distance =(distance + distance_) / 2;
    //   }
    // }
    // else
    // {
    //   distance = distance;
    // }
    // temp = distance;

    // 上一帧的distance
    double distance_;
    distance_ = points_distance_1[i];

    // // 方法一
    // // 计算当前帧distance与前一帧distance的差值
    // if (distance - distance_ > 0)
    // {
    //   if (distance - distance_ >=12)
    //   {
    //     distance = (1 + distance_ / (20 * (distance - distance_))) * distance_ + 1;
    //   }
    //   else
    //   {
    //     distance =(distance + distance_) / 2;
    //   }
    // }
    // else
    // {
    //   distance = distance;
    // }
    // points_distance_1[i] = distance;

    // 方法二
    if (distance - distance_ > 20)
    {
      // distance = 0.9 * distance_ + 0.1 * distance;
      distance = distance_ + 1.5 ;
    }
    else if (distance - distance_ > 0)
    {
      distance = 0.9 * distance_ + 0.1 * distance;
    }
    else
    {
      distance = distance;
    }
    points_distance_1[i] = distance;
 
   // 圆滑边界
  //  if (i > 0 && points_distance_1[i] - points_distance_1[i-1] > 5 
  //                && points_distance_1[i] - points_distance_1[i+1] > 5)
  //   {
  //     points_distance_1[i] = points_distance_1[i-1];
  //   }
    
    // std::ofstream writeFile;
    // writeFile.open("/home/wang/driveworks/freespace_lidar/src/handle/txt/distance.txt", ios_base::app);
    // writeFile << distance;
    // writeFile << "\n";
    // writeFile.close();

    PointI outPoint;
    get_point_position(theta, distance, outPoint);
    out_point_cloud->points.push_back(outPoint);
    
    //添加画图的原点
//    if (i == halfListSize - 1 && t_angle_free > 0)
//    {
//      PointI originPoint;
//      originPoint.x = 0.0;
//      originPoint.y = 0.0;
//      originPoint.z = 0.0;
//      originPoint.intensity = 255.0;
//      out_point_cloud->points.push_back(originPoint);
//    }
  }
//   std::ofstream writeFile;
//   writeFile.open("/home/wang/driveworks/freespace_lidar/src/handle/txt/points.txt", ios_base::app);
//   writeFile << out_point_cloud->points.back();
//   writeFile << "\n";
//   writeFile.close();
}

void FreeSpace::get_point_position(const double theta, const double distance, PointI &out_point)
{
  out_point.x = - distance * cos(theta * M_PI / 180.0);
  out_point.y = distance * sin(theta * M_PI / 180.0);
  out_point.z = 0.0;
  out_point.intensity = 255.0;
}

void FreeSpace::get_closest_distance_2(std::vector<std::vector<grid_feature> > &polar_grid_points,
                                       const std::vector<std::vector<int> > &objects_grid,
                                       const int nRows, const int nCols, std::vector<double> &point_list)
{
  for (int i = 0; i < nCols; i++)
  {
    for (int j = 0; j < nRows; j++)
    {
      if (objects_grid[j][i] == 1)
      {
//        if (objects_grid[j][i] == 1){
//          double closestDis = polar_grid_points[j][i].closestDis;
//          point_list.at(i) = closestDis;
//          break;
//        }
//        else
//        {
//          double closestDis = curb_points[j][i].closestDis;
//          point_list.at(i) = closestDis;
//          break;
//        }

        // 不使用curb
        double closestDis = polar_grid_points[j][i].closestDis;
        point_list.at(i) = (j + 0.5) * config_.distance_resolution + config_.distance_min;//closestDis;
        break;
      }
    }
  }
}

void FreeSpace::get_closest_distance(std::vector<std::vector<grid_feature> > &polar_grid_points,
                                     const std::vector<std::vector<int> > &objects_grid,
                                     const int nRows, const int nCols, std::vector<double> &point_list)
{
  double front_angle = 90 - atan(config_.distance_max / config_.t_side_distance) * 180  / M_PI;
  for (int i = 0; i < nCols; i++)
  {
    double theta = get_theta(i) + config_.angle_begin;
    PointI tmp;
    double t_dis;
    get_point_position(theta, config_.distance_max, tmp);
    if(tmp.y > 0)
      t_dis = 15 / tmp.y * config_.distance_max;
    if(tmp.y < 0)
      t_dis = -15 / tmp.y * config_.distance_max;

    if ((theta > 180 - front_angle && theta < 180 + front_angle) || (theta < front_angle) ||
        (theta > config_.angle_end - front_angle))
    {
      for (int j = 0; j < nRows; j++)
      {
        if (objects_grid[j][i] == 1)
        {
          double closestDis = polar_grid_points[j][i].closestDis;
          point_list.at(i) = closestDis;
          break;
        }
      }
    }
    else
    {
      for (int j = 0; j < nRows; j++)
      {
        if (objects_grid[j][i] == 1)
        {
          double closestDis = polar_grid_points[j][i].closestDis;
          if (closestDis < t_dis)
            point_list.at(i) = closestDis;
          else
            point_list.at(i) = t_dis;
          break;
        }
        if (j == nRows - 1)
        {
          point_list.at(i) = t_dis;
        }
      }
    }


  }
}

void FreeSpace::check_grid(const std::vector<std::vector<grid_feature> > &polar_grid_points,
                           const int rows, const int cols,
                           std::vector<std::vector<int> > &objects_grid)
{
  for (int i = 0; i < rows; i++)
  {
    for (int j = 0; j < cols; j++)
    {
      double pointsHeight = polar_grid_points[i][j].maxHeight - polar_grid_points[i][j].minHeight;
      double pointsNums = polar_grid_points[i][j].pointsNums;
      if (pointsHeight > config_.t_height &&  pointsNums > config_.t_nums)
      {
        objects_grid[i][j] = 1;
      }
    }
  }

}

bool FreeSpace::construct_polar_grid(const PointICloudPtr &point_cloud,
                                     std::vector<std::vector<grid_feature> > &polar_grid_points)
{
  int pointsSize = point_cloud->points.size();
  if (pointsSize == 0)
  {
    return false;
  }
  for(int i = 0; i < pointsSize; i++)
  {
    if (std::isnan(point_cloud->points[i].x))
      continue;
    //角度值
    double pointTheta = get_theta(point_cloud->points[i]);
    if (pointTheta <= config_.angle_begin || pointTheta >=  config_.angle_end)
      continue;
    int nCols = std::floor((pointTheta - config_.angle_begin) / config_.angle_resolution);
    //距离值
    double pointDistance = get_points_distance(point_cloud->points[i]);
    int nRows = std::floor((pointDistance - config_.distance_min) / config_.distance_resolution);

    if (pointDistance >= config_.distance_max || pointDistance < config_.distance_min)
    {
      continue;
    }
//    std::cout<<"dist: " << pointDistance <<std::endl;
//    std::cout<<"nRows: " << nRows << ", nCols" << nCols <<std::endl;
    //ROS_INFO("x %f, y %f", point_cloud->points[i].x , point_cloud->points[i].y);

    //ROS_INFO("rows : %d, cols : %d  dis %f, theta : %f", nRows, nCols, pointDistance, pointTheta);
    double pointZ = point_cloud->points[i].z;
    if (pointZ > polar_grid_points[nRows][nCols].maxHeight)
      polar_grid_points[nRows][nCols].maxHeight = pointZ;

    if (pointZ < polar_grid_points[nRows][nCols].minHeight){
        //ROS_INFO("height : %f, z %f, row: %d, col: %d", polar_grid_points[nRows][nCols].minHeight, pointZ,nRows,nCols);
      polar_grid_points[nRows][nCols].minHeight = pointZ;
    }

    if (pointDistance < polar_grid_points[nRows][nCols].closestDis)
      polar_grid_points[nRows][nCols].closestDis = pointDistance;
    polar_grid_points[nRows][nCols].pointsNums++;
  }


}

double FreeSpace::get_points_distance(const PointI &point)
{
  double distance = sqrt(point.x * point.x + point.y * point.y);
  return distance;
}

double FreeSpace::get_theta(const PointI &point)
{
  double angleDeg = atan2(point.y, point.x) / M_PI * 180.0;
  double refAngleDeg;
  refAngleDeg = 180.0 - angleDeg;
  assert(refAngleDeg >=0 && refAngleDeg <= 360);
  return refAngleDeg;
}

double FreeSpace::get_theta(const int i)
{
  return (i * config_.angle_resolution);
}

void FreeSpace::onPointCloud(const sensor_msgs::PointCloud2ConstPtr &point_cloud_msg,
                                  visualization_msgs::MarkerArray &free_space_marker,
                                  sensor_msgs::PointCloud2 &free_space_msg, const config fsconfig,
                                  lidar::RoiPoint roi_point)
{
  std_msgs::Header header = point_cloud_msg->header;

  pcl::console::TicToc time;

  time.tic();
  //初始化
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZI>);//点云指针
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterPtr1(new pcl::PointCloud<pcl::PointXYZI>);//滤波后的指针
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterPtr2(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterPtr3(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterPtr4(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterPtr5(new pcl::PointCloud<pcl::PointXYZI>);
  //转换成pointT
  pcl::fromROSMsg(*point_cloud_msg,*cloudPtr);//转换成xyzi

  // 直通滤波

  pcl::PassThrough<pcl::PointXYZI> pass1;
  //pcl::PassThrough<pcl::PCLPointCloud2> pass;
  pass1.setInputCloud (cloudPtr);
  pass1.setFilterFieldName ("x");
  pass1.setFilterLimits (-50.0, 50.0);
  pass1.setFilterLimitsNegative (false);
  pass1.filter (*filterPtr1);

  pcl::PassThrough<pcl::PointXYZI> pass2;
  pass2.setInputCloud(filterPtr1);
  pass2.setFilterFieldName ("y");
  pass2.setFilterLimits (-50.0, 50.0);
  pass2.setFilterLimitsNegative (false);
  pass2.filter (*filterPtr2);

  pcl::PassThrough<pcl::PointXYZI> pass3;
  pass3.setInputCloud(filterPtr2);
  pass3.setFilterFieldName ("z");
  pass3.setFilterLimits (-5.0, 2);
  pass3.setFilterLimitsNegative (false);
  pass3.filter (*filterPtr3);

  // 创建体素栅格下采样: 下采样的大小为20cm
  pcl::VoxelGrid<PointI> sor;  //体素栅格下采样对象
  sor.setInputCloud (filterPtr3);             //原始点云
  sor.setLeafSize (0.2f, 0.2f, 0.2f);    // 设置采样体素大小
  sor.setMinimumPointsNumberPerVoxel(1);
  sor.filter (*filterPtr4);        //保存

  pcl::RadiusOutlierRemoval<pcl::PointXYZI> outrem;  //创建滤波器
  outrem.setInputCloud(filterPtr4);    //设置输入点云filterPtr4
  outrem.setRadiusSearch(0.3);     //设置半径为0.8的范围内找临近点
  outrem.setMinNeighborsInRadius (5); //设置查询点的邻域点集数小于2的删除
   //apply filter
  outrem.filter (*filterPtr5);     //执行条件滤波   在半径为0.8 在此半径内必须要有两个邻居点，此点才会保存

  //################################### free space ##############################

  std::vector<int> point_list;
  PointICloudPtr free_space_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  PointICloudPtr out_point_cloud_xy(new pcl::PointCloud<pcl::PointXYZI>());

  config_ = fsconfig;
  get_free_space(header, filterPtr5, point_list, roi_point,
                 free_space_cloud, out_point_cloud_xy, free_space_marker);

  fs_points.clear();
  for (int i = 0; i < free_space_cloud->points.size(); i++)
  {
      pointxy point;
      point.x = free_space_cloud->points[i].x;
      point.y = free_space_cloud->points[i].y;
      fs_points.push_back(point);
  }
  // std::cout << "size " << fs_points.size() << std::endl;
  pcl::toROSMsg(*free_space_cloud, free_space_msg);
  free_space_msg.header = header;

}

void FreeSpace::onPointCloud_wangran(const sensor_msgs::PointCloud2ConstPtr &point_cloud_msg,
                                   sensor_msgs::PointCloud2 &free_space_msg)
{
  std_msgs::Header header = point_cloud_msg->header;

  pcl::console::TicToc time;

  time.tic();
  //初始化
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZI>);//点云指针
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterPtr1(new pcl::PointCloud<pcl::PointXYZI>);//滤波后的指针
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterPtr2(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterPtr3(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterPtr4(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterPtr5(new pcl::PointCloud<pcl::PointXYZI>);
  //转换成pointT
  pcl::fromROSMsg(*point_cloud_msg,*cloudPtr);//转换成xyzi

  // 直通滤波

  pcl::PassThrough<pcl::PointXYZI> pass3;
  pass3.setInputCloud(cloudPtr);
  pass3.setFilterFieldName ("z");
  pass3.setFilterLimits (-5.0, 0.1);
  pass3.setFilterLimitsNegative (false);
  pass3.filter (*filterPtr3);

  pcl::toROSMsg(*filterPtr3, free_space_msg);
  free_space_msg.header = header;

}


}
