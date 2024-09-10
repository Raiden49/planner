/*
 * @Author: Raiden49 
 * @Date: 2024-06-26 10:24:41 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-09-09 10:59:15
 */
#ifndef ASTAR_HPP_
#define ASTAR_HPP_

#include "global_planner/global_planner_interface.hpp"

namespace global_planner
{
/**
 * @brief A*作全局规划，没什么特殊的
 */
class AStar : public GlobalPlannerInterface {
    public:
        AStar(const Eigen::MatrixXi& map, const std::array<int, 2>& start, 
              const std::array<int, 2>& goal) : 
              GlobalPlannerInterface(map, start, goal) {
            close_set_map_ = Eigen::MatrixXi::Zero(map.rows(), map.cols());
            open_set_map_ = Eigen::MatrixXi::Zero(map.rows(), map.cols());
            cost_map_ = Eigen::MatrixXd::Zero(map.rows(), map.cols());
        }
        ~AStar() override = default;
        struct AstarNode
        {
            int x, y;
            double g_cost, h_cost, f_cost;
            std::shared_ptr<AstarNode> parent = nullptr;
            AstarNode(int x, int y) : x(x), y(y), g_cost(0), h_cost(0), f_cost(0) {};
            AstarNode() = default;
        };
        
        bool NeighborSearch(AstarNode& node);
        double GetNodeCost(const AstarNode& node);
        bool GetPath(AstarNode& node, std::vector<Point3d>& path);
        bool GetPlan(std::vector<Point3d>& path) override;
    private:
        std::vector<AstarNode> close_set_;
        std::vector<AstarNode> open_set_;
        Eigen::MatrixXi close_set_map_ = Eigen::Matrix<int, 1, 1>();
        Eigen::MatrixXi open_set_map_ = Eigen::Matrix<int, 1, 1>();
        Eigen::MatrixXd cost_map_ = Eigen::Matrix<double, 1, 1>();
};
}

#endif // ASTAR_HPP_