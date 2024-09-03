/*
 * @Author: Raiden49 
 * @Date: 2024-09-03 15:38:29 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-09-03 16:05:50
 */
#ifndef RRT_STAR_HPP_
#define RRT_STAR_HPP_

#include "global_planner/rrt.hpp"

namespace global_planner
{
class RRTStar : public RRT {
    public:
        RRTStar(const Eigen::MatrixXi& map, const std::array<int, 2>& start, 
                const std::array<int, 2>& goal) : RRT(map, start, goal) {
            
            used_map_ = Eigen::MatrixXi::Zero(map.rows(), map.cols());
            used_map_(start.at(0), start.at(1)) = 1;
            // used_map_(goal.at(0), goal.at(1)) = 1;
            rrt_tree_.push_back(RRTNode(start.at(0), start.at(1)));
        }
        ~RRTStar() override = default;
        bool AddNewNodeToRRTStarTree(RRTNode& temp_node);
        bool GetPlan(std::vector<Point3d>& path) override;
    private:
        double rewrite_thread_ = 2.0 * step_size_; 
        double relink_thread_ = 2.0 * step_size_;
};
}

#endif // RRT_STAR_HPP_