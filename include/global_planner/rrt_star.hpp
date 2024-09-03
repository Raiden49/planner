/*
 * @Author: Raiden49 
 * @Date: 2024-09-03 15:38:29 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-09-03 17:13:17
 */
#ifndef RRT_STAR_HPP_
#define RRT_STAR_HPP_

#include "global_planner/rrt.hpp"

namespace global_planner
{
class RRTStar : public RRT {
    public:
        RRTStar(const Eigen::MatrixXi& map, const std::array<int, 2>& start, 
                const std::array<int, 2>& goal, const double& step_size,
                const double& rewrite_thread, const double& relink_thread) : 
                rewrite_thread_(rewrite_thread * step_size),
                relink_thread_(relink_thread * step_size),
                RRT(map, start, goal, step_size) {
            
            used_map_ = Eigen::MatrixXi::Zero(map.rows(), map.cols());
            used_map_(start.at(0), start.at(1)) = 1;
            // used_map_(goal.at(0), goal.at(1)) = 1;
            rrt_tree_.push_back(RRTNode(start.at(0), start.at(1)));
        }
        ~RRTStar() override = default;
        bool AddNewNodeToRRTStarTree(RRTNode& temp_node);
        bool GetPlan(std::vector<Point3d>& path) override;
    private:
        double step_size_;
        double rewrite_thread_; 
        double relink_thread_;
};
}

#endif // RRT_STAR_HPP_