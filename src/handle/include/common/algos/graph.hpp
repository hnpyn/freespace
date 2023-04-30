#ifndef COMMON_LIBS_INCLUDE_COMMON_ALGOS_GRAPH_HPP_
#define COMMON_LIBS_INCLUDE_COMMON_ALGOS_GRAPH_HPP_

#include <queue>
#include <vector>

namespace lidar {
namespace common {
namespace algos {

// 基于BFS的连通分量分析 ==> 图是连通的？
void connectedComponentAnalysis(const std::vector<std::vector<int>>& graph,
                                std::vector<std::vector<int>>* components) {
    int num_item = graph.size();
    std::vector<int> visited;
    visited.resize(num_item, 0);
    std::queue<int> que;
    std::vector<int> component;
    components->clear();

    for (int i = 0; i < num_item; i++) {
        if (visited[i]) {
            continue;
        }
        component.push_back(i);
        que.push(i);
        visited[i] = 1;
        while (!que.empty()) {
            int id = que.front();
            que.pop();
            for (size_t j = 0; j < graph[id].size(); j++) {
                int nb_id = graph[id][j];
                if (visited[nb_id] == 0) {
                    component.push_back(nb_id);
                    que.push(nb_id);
                    visited[nb_id] = 1;
                }
            }
        }
        components->push_back(component);
        component.clear();
    }
}

}  // namespace algos
}  // namespace common
}  // namespace lidar

#endif  // COMMON_LIBS_INCLUDE_COMMON_ALGOS_GRAPH_HPP_
