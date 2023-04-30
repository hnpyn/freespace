#include "fusion.h"

fusion::fusion()
{

}

bool fusion::input_points(pcl::PointCloud<pcl::PointXYZI> &lidarPoints,
                         pcl::PointCloud<pcl::PointXYZI> &cameraPoints)
{
    points_from_lidar_ = lidarPoints;
    points_from_camera_ = cameraPoints;
    points_lidar_nums_ = points_from_lidar_.points.size();
    points_camera_nums_ = points_from_camera_.points.size();
    if (0 == points_lidar_nums_){
        ROS_INFO("NO LIDAR POINTS !");
        return false;
    }
    else if (0 == points_camera_nums_){
        ROS_INFO("NO CAMERA POINTS!");
        return false;
    }
    return true;
}

void fusion::set_model(int model)
{
    model_ = model;
}

bool fusion::on_fusion()
{
    switch (model_) {
    case LIDAR:
        pcl::PointCloud<pcl::PointXYZI> outPointCloud;
        encode_single_sensor(points_from_lidar_, points_lidar_nums_,
                             outPointCloud);

        break;

    case CAMERA:
        encode_single_sensor(points_from_camera_, points_camera_nums_);
        break;

    case FUSION:
        break;
    default:
        break;
    }
}

void fusion::encode_single_sensor(pcl::PointCloud<pcl::PointXYZI> &pointCloud, int nums,
                                  pcl::PointCloud<pcl::PointXYZI> &outPointCloud)
{

}
