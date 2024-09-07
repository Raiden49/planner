/*
 * @Author: Raiden49 
 * @Date: 2024-09-06 15:40:45 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-09-06 15:47:37
 */
#include "global_planner/jps.hpp"

namespace global_planner
{
std::shared_ptr<AStar::AstarNode> JPS::GetJumpPoint(int x, int y, int dx, int dy,
                                                    const AstarNode& start_node,
                                                    const AstarNode& goal_node) {
    if (!map_tool_->InBoundary(x, y) || map_tool_->HasObstacle(x, y)) {
        return nullptr;
    }
    if (x == goal_node.x && y == goal_node.y) {
        return std::make_shared<AstarNode>(x, y);
    }
    
    if (dx && dy) {
        if ((map_tool_->InBoundary(x - dx, y + dy) && map_tool_->HasObstacle(x - dx, y + dy)) || 
            (map_tool_->InBoundary(x + dx, y - dy) && map_tool_->HasObstacle(x + dx, y - dy))) {
            return std::make_shared<AstarNode>(x, y);            
        }
        if (GetJumpPoint(x + dx, y, dx, 0, start_node, goal_node) || 
            GetJumpPoint(x, y + dy, 0, dy, start_node, goal_node)) {
            return std::make_shared<AstarNode>(x, y);
        }
    }
    else {
        if (dx) {
            if ((map_tool_->InBoundary(x + dx, y + 1) && map_tool_->HasObstacle(x + dx, y + 1)) || 
                (map_tool_->InBoundary(x + dx, y - 1) && map_tool_->HasObstacle(x + dx, y - 1))) {
                return std::make_shared<AstarNode>(x, y);
            }
        }
        else if (dy) {
            if ((map_tool_->InBoundary(x + 1, y + dy) && map_tool_->HasObstacle(x + 1, y + dy)) || 
                (map_tool_->InBoundary(x - 1, y + dy) && map_tool_->HasObstacle(x - 1, y + dy))) {
                return std::make_shared<AstarNode>(x, y);
            }
        }
    }
    return GetJumpPoint(x + dx, y + dy, dx, dy, start_node, goal_node);
}
bool JPS::GetPlan(std::vector<Point3d>& path) {
    auto start_node = AstarNode(start_[0], start_[1]);
    auto goal_node = AstarNode(goal_[0], goal_[1]);
    start_node.f_cost = 0; start_node.parent = nullptr;
    open_list_.push(std::make_pair(start_node.f_cost, start_node));
    searched_map_(start_node.x, start_node.y) = 1;

    while (!open_list_.empty()) {
        auto node = std::make_shared<AstarNode>(open_list_.top().second);
        open_list_.pop();
        if (node->x == goal_node.x && node->y == goal_node.y) {
            while (node) {
                path.push_back(Point3d(map_tool_->Map2World(node->x, node->y)));
                node = node->parent;
            }
            std::reverse(path.begin(), path.end());
            return true;
        }
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                if (dx == 0 && dy == 0) {
                    continue;
                }
                auto jump_point = GetJumpPoint(node->x + dx, node->y + dy, 
                                               dx, dy, start_node, goal_node);
                if (jump_point) {
                    if (!searched_map_(jump_point->x, jump_point->y)) {
                        jump_point->g_cost = node->g_cost + m_util::EuclideanDis(
                                node->x, node->y, jump_point->x, jump_point->y);
                        jump_point->h_cost = m_util::EuclideanDis(
                                node->x, node->y, goal_node.x, goal_node.y);
                        jump_point->f_cost = jump_point->g_cost + jump_point->h_cost;
                        jump_point->parent = node;
                        open_list_.push(std::make_pair(jump_point->f_cost, *jump_point));
                        searched_map_(jump_point->x, jump_point->y) = 1;
                    }
                }
            }
        }
    }
    return false;
}
}