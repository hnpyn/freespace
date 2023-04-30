#ifndef VISUALIZE_H
#define VISUALIZE_H

#include <visualization_msgs/MarkerArray.h>
#include "src/HD/HDmap/tools/mapType.h"

namespace lidarPerception {

class visualize
{
public:
  visualize();

  void handle(const HdmapStruct &map);

  void get(visualization_msgs::MarkerArray &markerArray);

private:
  void markerS(const pointxyz &signs,
              const int speed,
              visualization_msgs::Marker &marker);

  void markerR(const std::pair<pointRGBACloudPtr, std::string> &pair,
                          visualization_msgs::Marker &marker);

  void markerC(int flags, const pointRGBACloudPtr cloud,
                          visualization_msgs::Marker &marker,
                          visualization_msgs::Marker &markerTxt);

private:
  visualization_msgs::MarkerArray markerArray_;
};
}


#endif // VISUALIZE_H
