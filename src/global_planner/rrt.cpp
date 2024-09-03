/*
 * @Author: Raiden49 
 * @Date: 2024-08-28 17:22:42 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-09-03 16:37:32
 */
#include "global_planner/rrt.hpp"

namespace global_planner
{
int RRT::GetNearestNodeId(const double& x, const double& y) {
    int id = 0;
    double dis = DBL_MAX;
    for (int i = 0; i < this->rrt_tree_.size(); i++) {
        double temp_dis = m_util::EuclideanDis(x, y, rrt_tree_[i].x, rrt_tree_[i].y);
        if (temp_dis < dis) {
            dis = temp_dis;
            id = i;
        }
    }
    return id;
}
bool RRT::GeneratePoints(RRTNode& temp_node) {
    srand((unsigned) time(NULL));
    int count = map_ptr_->rows() * map_ptr_->cols();
    while (count--) {
        int x = rand() % map_ptr_->rows();
        int y = rand() % map_ptr_->cols();
        if (used_map_(x, y) == 1) {
            continue;
        }
        used_map_(x, y) = 1;
        temp_node.x = x; temp_node.y = y;
        return true;
    }
    ROS_INFO("Generate new poins failed!!!");
    return false;
}
bool RRT::AddNewNodeToRRTTree(RRTNode& temp_node) {
    auto nearest_node = rrt_tree_[GetNearestNodeId(temp_node.x, temp_node.y)];
    double theta = atan2(temp_node.y - nearest_node.y, temp_node.x - nearest_node.x);
    double x = nearest_node.x + (step_size_ * cos(theta));
    double y = nearest_node.y + (step_size_ * sin(theta));

    if (IsPointValid(x, y)) {
        temp_node.x = x; temp_node.y = y; 
        temp_node.parent = std::make_shared<RRTNode>(nearest_node);
        temp_node.parent->children_nodes.push_back(temp_node);
        rrt_tree_.push_back(temp_node);
        return true;
    }
    return false;
}
bool RRT::IsPointValid(const double&x, const double& y) {
    if (x > map_ptr_->rows() || x < 0) {
        return false;
    }
    if (y > map_ptr_->cols() || y < 0) {
        return false;
    }
    if ((*map_ptr_)((int)x, (int)y) > 0) {
        return false;
    }
    return true;
}
bool RRT::IsGoalReached(const RRTNode& temp_node) {
    double dis = m_util::EuclideanDis(temp_node.x, temp_node.y, 
                                      goal_.at(0), goal_.at(1));
    if (dis <= 2.0) {
        rrt_tree_.push_back(RRTNode(goal_.at(0), goal_.at(1), temp_node));
        return true;
    }
    return false;
}
bool RRT::GetPlan(std::vector<Point3d>& path) {
    if ((*map_ptr_)(start_.at(0), start_.at(1)) > 0) {
        std::cout << "start point is valid(in obstacles)" << std::endl;
        return false;
    }

    RRTNode temp_node(0, 0);
    int iter_count = 50000;
    while (iter_count--) {
        if (GeneratePoints(temp_node)) {
            if (AddNewNodeToRRTTree(temp_node)) {
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
} // namespace global_planner
