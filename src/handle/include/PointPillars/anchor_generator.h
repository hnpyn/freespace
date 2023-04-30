#ifndef ANCHOR_GENERATOR_H
#define ANCHOR_GENERATOR_H
#include <iostream>
#include <map>
#include <vector>

class AnchorGenerator
{
public:
  AnchorGenerator(const std::vector<float> &point_cloud_range,
                  std::map<std::string, std::map<std::string, std::vector<float>>> anchor_config);
  float* GenerateAnchors();
  inline void DestroyAnchors() {
    if (anchors_ptr_)
      delete [] anchors_ptr_;
  }
private:
  std::vector<float> anchor_range_;
  std::vector<std::vector<float>> anchor_size_;
  std::vector<std::vector<float>> anchor_rotation_;
  std::vector<std::vector<float>> anchor_height_;
  std::vector<std::vector<float>> feature_map_size_;
  float* anchors_ptr_;
};

#endif // ANCHOR_GENERATOR_H
