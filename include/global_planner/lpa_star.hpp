/*
 * @Author: Raiden49 
 * @Date: 2024-09-09 10:58:11 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-09-09 18:07:38
 */
#ifndef LPA_STAR_HPP_
#define LPA_STAR_HPP_

#include "global_planner/astar.hpp"

namespace global_planner 
{
class LPAStar : public AStar {
    public:
        LPAStar(const Eigen::MatrixXi& map, const std::array<int, 2>& start, 
                const std::array<int, 2>& goal) : AStar(map, start, goal) {
                g_map_ = Eigen::MatrixXd::Constant(map.rows(), map.cols(), DBL_MAX);
                rhs_map_ = Eigen::MatrixXd::Zero(map.rows(), map.cols());
                searched_map_ = Eigen::MatrixXi::Zero(map.rows(), map.cols());
        }
        ~LPAStar() override = default;
        std::vector<AstarNode> GetNeighbors(const AstarNode& node);
        bool UpdateVertex(const AstarNode& node);
        void ComputeShortestPath();
        double CalculateKey(const AstarNode& node);
        double GetCost(const AstarNode& node1, const AstarNode& node2);
        bool GetPlan(std::vector<Point3d>& path) override;
    private:
        struct Compare {
            bool operator()(const double& a, const double& b) {
                return a < b;
            }
        };
        std::multimap<double, AstarNode, Compare> u_list_;
        Eigen::MatrixXd rhs_map_, g_map_;
        Eigen::MatrixXi searched_map_;
};
}

#endif // LPA_STAR_HPP_