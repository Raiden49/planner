/*
 * @Author: Raiden49 
 * @Date: 2024-09-03 15:52:56 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-09-03 16:37:14
 */
#include "global_planner/rrt_star.hpp"

namespace global_planner 
{
bool RRTStar::AddNewNodeToRRTStarTree(RRTNode& temp_node) {
    auto nearest_node = rrt_tree_[GetNearestNodeId(temp_node.x, temp_node.y)];
    double theta = atan2(temp_node.y - nearest_node.y, temp_node.x - nearest_node.x);
    double x = nearest_node.x + (step_size_ * cos(theta));
    double y = nearest_node.y + (step_size_ * sin(theta));
    
    if (IsPointValid(x, y)) {
        temp_node.x = x; temp_node.y = y;
        temp_node.parent = std::make_shared<RRTNode>(nearest_node);
        std::vector<RRTNode> candidate_parents;
        double cost = temp_node.parent->cost;
        for (auto& node : rrt_tree_) {
            double cur_dis = 
                    m_util::EuclideanDis(temp_node.x, temp_node.y, node.x, node.y);
            if (cur_dis >= rewrite_thread_) {
                continue;
            }
            candidate_parents.push_back(node);
            if (node.cost < cost) {
                cost = node.cost;
                temp_node.cost = cost + cur_dis;
                temp_node.parent = std::make_shared<RRTNode>(node);
            }
        }
        for (auto& node : candidate_parents) {
            double cur_dis = 
                    m_util::EuclideanDis(temp_node.x, temp_node.y, node.x, node.y);
            if (cur_dis >= relink_thread_) {
                continue;
            }
            if (node.cost > cur_dis + temp_node.cost) {
                node.parent = std::make_shared<RRTNode>(temp_node);
                node.cost = cur_dis + temp_node.cost;
                for (auto& child_node : node.children_nodes) {
                    double child_dis = m_util::EuclideanDis(child_node.x, 
                                                            child_node.y, 
                                                            node.x, node.y);
                    child_node.cost = node.cost + child_dis;
                }
            }
        }
        temp_node.parent->children_nodes.push_back(temp_node);
        rrt_tree_.push_back(temp_node);
        return true;
    }
    return false;
}
bool RRTStar::GetPlan(std::vector<Point3d>& path) {
    if ((*map_ptr_)(start_.at(0), start_.at(1)) > 0) {
        std::cout << "start point is valid(in obstacles)" << std::endl;
        return false;
    }
    RRTNode temp_node(0, 0, 0);
    int iter_count = 50000;
    while (iter_count--) {
        if (GeneratePoints(temp_node)) {
            if (AddNewNodeToRRTStarTree(temp_node)) {
                if (IsGoalReached(temp_node)) {
                    auto node_ptr = std::make_shared<RRTNode>(rrt_tree_.back());
                    while (node_ptr) {
                        path.push_back(Point3d(Map2World(node_ptr->x, node_ptr->y)));
                        node_ptr = node_ptr->parent;
                    }
                    std::reverse(path.begin(), path.end());
                    return true;
                }
            }
        }
    }
    return false;
}
}