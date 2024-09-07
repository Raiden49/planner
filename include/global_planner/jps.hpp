/*
 * @Author: Raiden49 
 * @Date: 2024-09-06 15:32:11 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-09-06 15:44:43
 */
#ifndef JPS_HPP_
#define JPS_HPP_

#include "global_planner/astar.hpp"

namespace global_planner 
{
class JPS : public AStar {
    public:
        JPS(const Eigen::MatrixXi& map, const std::array<int, 2>& start, 
            const std::array<int, 2>& goal) : AStar(map, start, goal) {
            searched_map_ = Eigen::MatrixXi::Zero(map.rows(), map.cols());
        }
        ~JPS() override = default;
        std::shared_ptr<AstarNode> GetJumpPoint(int x, int y, int dx, int dy,
                                                const AstarNode& start_node,
                                                const AstarNode& goal_node);
        bool GetPlan(std::vector<Point3d>& path) override;
    private:
        struct Compare {
            bool operator()(const std::pair<double, AstarNode>& a, 
                            const std::pair<double, AstarNode>& b) {
                return a.first > b.first;
            }
        };
        // std::multimap<double, AstarNode> open_list_;
        std::priority_queue<std::pair<double, AstarNode>, 
                            std::vector<std::pair<double, AstarNode>>, 
                            Compare> open_list_;
        Eigen::MatrixXi searched_map_;
};
}

#endif // JPS_HPP_