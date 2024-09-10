/*
 * @Author: Raiden49 
 * @Date: 2024-09-09 11:25:08 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-09-09 18:09:53
 */
#include "global_planner/lpa_star.hpp"

namespace global_planner
{
double LPAStar::CalculateKey(const AstarNode& node) {
    double dis = m_util::EuclideanDis(node.x, node.y, goal_[0], goal_[1]);
    double key2 = std::min(g_map_(node.x, node.y), rhs_map_(node.x, node.y));
    double key1 = dis + key2;
    return key1 + key2;
}
double LPAStar::GetCost(const AstarNode& node1, const AstarNode& node2) {
    return m_util::EuclideanDis(node1.x, node1.y, node2.x, node2.y);
}
std::vector<AStar::AstarNode> LPAStar::GetNeighbors(const AstarNode& node) {
    std::vector<AstarNode> neighbor_nodes;
    std::vector<std::array<int, 2>> dirs = 
            {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
    for (auto& dir : dirs) {
        int x = node.x + dir[0];
        int y = node.y + dir[1];
        if (map_tool_->InBoundary(x, y) && !map_tool_->HasObstacle(x, y)) {
            auto neighbor_node = AstarNode(x, y);
            neighbor_nodes.push_back(neighbor_node);
        }
    }
    return neighbor_nodes;
}
void LPAStar::ComputeShortestPath() {
    while (!u_list_.empty()) {
        if (rhs_map_(goal_[0], goal_[1]) == g_map_(goal_[0], goal_[1])) {
            break;
        }
        auto node = u_list_.begin()->second;
        u_list_.erase(u_list_.begin());
        if (g_map_(node.x, node.y) > rhs_map_(node.x, node.y)) {
            g_map_(node.x, node.y) = rhs_map_(node.x, node.y);
            for (auto& neighbor_node : GetNeighbors(node)) {
                UpdateVertex(neighbor_node);
            }
        }
        else {
            g_map_(node.x, node.y) = DBL_MAX;
            UpdateVertex(node);
            for (auto& neighbor_node : GetNeighbors(node)) {
                UpdateVertex(neighbor_node);
            }
        }
    }
}
bool LPAStar::UpdateVertex(const AstarNode& node) {
    if (node.x != start_[0] || node.y != start_[1]) {
        double min_rhs = DBL_MAX;
        for (auto& neighbor : GetNeighbors(node)) {
            min_rhs = std::min(min_rhs, 
                    g_map_(neighbor.x, neighbor.y) + GetCost(node, neighbor));
        }
        rhs_map_(node.x, node.y) = min_rhs;
    }
    for (auto iter = u_list_.begin(); iter != u_list_.end(); ) {
        if (iter->second.x == node.x && iter->second.y == node.y) {
            iter = u_list_.erase(iter);
        }
        else {
            iter++;
        }
    }
    if (g_map_(node.x, node.y) != rhs_map_(node.x, node.y)) {
        u_list_.insert({CalculateKey(node), node});
    }
   
    return true;
}
bool LPAStar::GetPlan(std::vector<Point3d>& path) {
    auto start_node = AstarNode(start_[0], start_[1]);
    u_list_.insert({CalculateKey(start_node), start_node});
    rhs_map_(start_node.x, start_node.y) = 0;
    // 如果地图更新了，应先调用UpdateVertex函数把更新后节点加入到u_list中，然后再调用该函数
    ComputeShortestPath();

    auto node = AstarNode(goal_[0], goal_[1]);
    while (node.x != start_node.x && node.y != start_node.y) {
        path.push_back(Point3d(map_tool_->Map2World(node.x, node.y)));
        auto prev_node = node;
        double min_cost = DBL_MAX;
        for (auto& neighbor : GetNeighbors(node)) {
            double cost = g_map_(neighbor.x, neighbor.y);
            if (cost < min_cost) {
                min_cost = cost;
                prev_node = neighbor;
            }
        }
        node = prev_node;       
    }
    path.push_back(Point3d(map_tool_->Map2World(start_node.x, start_node.y)));
    std::reverse(path.begin(), path.end());
    return true;
}
}