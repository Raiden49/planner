/*
 * @Author: Raiden49 
 * @Date: 2024-08-28 16:43:55 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-09-03 17:01:43
 */
#ifndef RRT_HPP_
#define RRT_HPP_

#include "global_planner/global_planner_interface.hpp"

namespace global_planner 
{
class RRT : public GlobalPlannerInterface {
    public:
        RRT(const Eigen::MatrixXi& map, const std::array<int, 2>& start, 
            const std::array<int, 2>& goal, const double& step_size) : 
            step_size_(step_size), GlobalPlannerInterface(map, start, goal) {
            
            used_map_ = Eigen::MatrixXi::Zero(map.rows(), map.cols());
            used_map_(start.at(0), start.at(1)) = 1;
            // used_map_(goal.at(0), goal.at(1)) = 1;
            rrt_tree_.push_back(RRTNode(start.at(0), start.at(1)));
        }
        ~RRT() override = default;
        
        struct RRTNode {
            double x;
            double y;
            double cost;
            std::shared_ptr<RRTNode> parent;
            std::vector<RRTNode> children_nodes;
            RRTNode(const double& x, const double& y) : 
                    x(x), y(y), cost(0), parent(nullptr) {};
            RRTNode(const double& x, const double& y, const RRTNode& node) : 
                    x(x), y(y), cost(0), parent(std::make_shared<RRTNode>(node)) {}; 
            RRTNode(const double& x, const double& y, const double& cost) : 
                    x(x), y(y), cost(cost), parent(nullptr) {};
            RRTNode(const double& x, const double& y, const double& cost, const RRTNode& node) : 
                    x(x), y(y), cost(cost), parent(std::make_shared<RRTNode>(node)) {};
        };
        int GetNearestNodeId(const double& x, const double& y);
        bool GeneratePoints(RRTNode& temp_node);
        bool IsPointValid(const double&x, const double& y);
        bool IsGoalReached(const RRTNode& temp_node);
        bool AddNewNodeToRRTTree(RRTNode& temp_node);
        bool GetPlan(std::vector<Point3d>& path) override;
    public:
        double step_size_ = 1.0;
        std::vector<RRTNode> rrt_tree_;
        Eigen::MatrixXi used_map_;
};
}

#endif // RRT_HPP_