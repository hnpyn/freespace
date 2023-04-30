#include <string>

#include "LocalMap/local_map_filter.h"


std::string txtFilePath = "/home/wang/driveworks/map/picking_list.txt";

int
main (int argc, char** argv)
{
  // Local map ROI
  lidar::LocalMapFilter local_map_filter;
  local_map_filter.LoadLocalMap(txtFilePath);
}
