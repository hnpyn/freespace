#ifndef FUSION_H
#define FUSION_H
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <ros/ros.h>


enum FUSION_MODEL{LIDAR, CAMERA, FUSION};
class fusion
{
public:
    fusion();
    bool input_points(pcl::PointCloud<pcl::PointXYZI> &lidarPoints,
                     pcl::PointCloud<pcl::PointXYZI> &cameraPoints);

    void set_model(int model);

    bool on_fusion();

private:
    void encode_single_sensor(pcl::PointCloud<pcl::PointXYZI> &pointCloud, int nums, pcl::PointCloud<pcl::PointXYZI> &outPointCloud);


private:
    pcl::PointCloud<pcl::PointXYZI> points_from_lidar_;
    pcl::PointCloud<pcl::PointXYZI> points_from_camera_;
    int points_lidar_nums_;
    int points_camera_nums_;
    FUSION_MODEL model_;
};

#endif // FUSION_H
