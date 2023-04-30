#include "config.h"

config::config(const string &config_path)
{
    if (!load_config(config_path)) {
        printf("[ERROR] Load configuration file failed!\n");
        exit(1);
    }
    printf("Load configuration file success!\n");
}

bool config::load_config(const string &config_path){
    cv::FileStorage fs;
    fs.open(config_path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        printf("Failed to open the config file: %s\n", config_path.c_str());
        return false;
    }
    distance_max = (float)fs["distance_max"];
    distance_min = (float)fs["distance_min"];
    distance_resolution = (float)fs["distance_resolution"];

    angle_begin = (float)fs["angle_begin"];
    angle_end = (float)fs["angle_end"];
    angle_resolution = (float)fs["angle_resolution"];

    t_height = (float)fs["t_height"];
    t_nums = (float)fs["t_nums"];


    t_correct_dis = (float)fs["t_correct_dis"];
    t_angle_begin = (float)fs["t_angle_begin"];
    t_angle_end = (float)fs["t_angle_end"];

    t_side_distance = (float)fs["t_side_distance"];
    t_angle_free = (float)fs["t_angle_free"];

    return true;
}
