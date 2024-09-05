/*
 * @Author: Raiden49 
 * @Date: 2024-06-26 10:26:19 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-08-15 15:23:09
 */
#include "global_planner/astar.hpp"

namespace global_planner
{

double AStar::GetNodeCost(const AstarNode& node) {
    return m_util::EuclideanDis(node.x, node.y, start_[0], start_[1]) + 
           m_util::EuclideanDis(node.x, node.y, goal_[0], goal_[1]);
}

bool AStar::NeighborSearch(AstarNode& node) {
    for (int i = -1; i < 2; i++) {

        if (node.x + i >= map_ptr_->rows() || node.x + i < 0) {
            continue;
        }
        for (int j = -1; j < 2; j++) {

            if (i == 0 && j == 0) {
                continue;
            }

            if (node.y + j >= map_ptr_->cols() || node.y + j < 0) {
                continue;
            }

            AstarNode temp_node;
            temp_node.x = node.x + i;
            temp_node.y = node.y + j;
            temp_node.parent = std::make_shared<AstarNode>(node);
            temp_node.g_cost = node.g_cost + sqrt(i * i + j * j);
            temp_node.h_cost = 
                    m_util::EuclideanDis(node.x, node.y, goal_[0], goal_[1]);
            temp_node.f_cost = temp_node.g_cost + temp_node.h_cost;
            
            if (temp_node.x == goal_[0] && temp_node.y == goal_[1]) {
                open_set_.push_back(temp_node);
                return true;
            }        

            if ((*map_ptr_)(node.x + i, node.y + j) > 0) {
                continue;
            }
            if (close_set_map_(node.x + i, node.y + j)) {
                continue;
            } 
            else if (open_set_map_(node.x + i, node.y + j)) {
                for (auto iter_node : open_set_) {
                    if (iter_node.x == node.x + i && iter_node.y == node.y + j) {
                        if (iter_node.g_cost > temp_node.g_cost) {
                            iter_node.parent = std::make_shared<AstarNode>(node);
                            iter_node.g_cost = temp_node.g_cost;
                            iter_node.f_cost = 
                                    temp_node.g_cost + temp_node.h_cost;
                        }
                    }
                }
            }
            else {
                open_set_.push_back(temp_node);
                open_set_map_(temp_node.x, temp_node.y) = 1;
            }
        }
    }

    return false;
}

bool AStar::GetPath(AstarNode& node, std::vector<Point3d>& path) {
    AstarNode current_node = node;
    while (1) {
        if (current_node.x == start_[0] && current_node.y == start_[1]) {
            break;
        }
        path.push_back(map_tool_->Map2World(current_node.x, current_node.y));
        current_node = *current_node.parent;
    }
    std::reverse(path.begin(), path.end());
    return true;
}

bool AStar::GetPlan(std::vector<Point3d>& path) {
    AstarNode start_node;
    start_node.parent = std::make_shared<AstarNode>(start_node);;
    start_node.x = start_[0]; start_node.y = start_[1];
    start_node.g_cost = 0, start_node.h_cost = 0, start_node.f_cost = 0;

    if ((*map_ptr_)(start_node.x, start_node.y) > 0) {
        std::cout << "start point is valid(in obstacles)" << std::endl;
        return false;
    }

    open_set_.push_back(start_node);
    open_set_map_(start_node.x, start_node.y) = 1;

    AstarNode current_node = open_set_[0];
    while (1) {
        int min_cost = INFINITY, num = 0;
        for (int i = 0; i < open_set_.size(); i++) {
            if (open_set_[i].f_cost < min_cost) {
                min_cost = open_set_[i].f_cost;
                num = i;
            }
        }
        current_node = open_set_[num];
        if (current_node.x == goal_[0] && current_node.y == goal_[1]) {
            break;
        }
        close_set_.push_back(current_node);
        close_set_map_(current_node.x, current_node.y) = 1;
        open_set_.erase(open_set_.begin() + num);
        open_set_map_(current_node.x, current_node.y) = 0;

        if (NeighborSearch(current_node)) {
            current_node = open_set_[open_set_.size() - 1];
            break;
        }

    }

    if (GetPath(current_node, path)) {
        return true;
    }

    return false;
}
}