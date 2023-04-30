#ifndef MODULES_PERCEPTION_OBSTACLE_COMMON_GRAPH_UTIL_H_
#define MODULES_PERCEPTION_OBSTACLE_COMMON_GRAPH_UTIL_H_

#include <vector>

namespace lidar {

// bfs based component analysis
void ConnectedComponentAnalysis(const std::vector<std::vector<int>>& graph,
                                std::vector<std::vector<int>>* components);

}  // namespace lidar

#endif  // MODULES_PERCEPTION_OBSTACLE_COMMON_GRAPH_UTIL_H_
