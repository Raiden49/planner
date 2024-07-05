/*
 * @Author: Raiden49 
 * @Date: 2024-06-26 10:25:07 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-06-26 11:16:14
 */
#ifndef ONLINE_LOCAL_PLANNER_HPP_
#define ONLINE_LOCAL_PLANNER_HPP_

#include "local_planner_interface.hpp"

namespace local_planner
{
/**
 * @brief 在线局部规划方法，使用五次多项式拟合几十条路径．但其实速度甚至比离线的还要慢，
 * Eigen计算需要优化一下，但是暂时懒得优化有点．．．
 */
class OnlineLocalPlanner : public LocalPlannerInterface {
    public:
        OnlineLocalPlanner(
                const Eigen::MatrixXi& map, 
                const std::vector<std::array<double, 2>>& ref_path) :
                LocalPlannerInterface(map, ref_path) {};
        ~OnlineLocalPlanner() = default;
        
        std::vector<std::vector<std::array<double, 2>>> Process(
                const double& radius,
                const std::array<double, 2>& pos, 
                const std::array<double, 2>& pos_ahead, 
                std::vector<std::array<double, 2>>& destination) override;
};
}

#endif // ONLINE_LOCAL_PLANNER_HPP_
