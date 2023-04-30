#ifndef CONFIG_H
#define CONFIG_H

#include <opencv/cv.h>
#include <string>
#include <stdio.h>

using namespace std;

class config
{
public:
  config() {}
  config(const string &config_path);
  bool load_config(const string &config_path);

public:
  float distance_max = 30.0;
  float distance_min = 5.0;
  float distance_resolution = 0.25;
  float angle_begin = 0.0;
  float angle_end = 360.0;
  float angle_resolution = 1;
  float t_height = 0.15;
  float t_nums = 0.0;
  float t_correct_dis = 4.0;//优化距离
  float t_angle_begin = 90;//freespace范围起始角度
  float t_angle_end = 270;//freespace范围中止角度
  float t_side_distance = 15.0;//freespace 两边距离
  float t_angle_free = 0.0;//freespace 中间空出的角度
};


#endif // CONFIG_H
