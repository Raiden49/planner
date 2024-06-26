/*
 * @Author: Raiden49 
 * @Date: 2024-06-26 10:24:38 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-06-26 10:56:49
 */
#ifndef GLOBAL_PLAN_INTERFACE_HPP_
#define GLOBAL_PLAN_INTERFACE_HPP_

#include <Eigen/Dense>
#include <vector>
#include <iostream>

#include "m_util.hpp"

namespace global_planner
{
/**
 * @brief 全局规划父类，纯接口类，无函数实现
 */
class GlobalPlannerInterface {
    public:
        GlobalPlannerInterface(const Eigen::MatrixXi& map,
                const std::array<int, 2>& start, 
                const std::array<int, 2>& goal) : 
                start_(start), goal_(goal) {
            map_ptr_ = std::make_shared<Eigen::MatrixXi>(map);
        }
        virtual ~GlobalPlannerInterface() {};
        virtual bool GetPlan(std::vector<std::array<int, 2>>& path) = 0;
    public:
        std::shared_ptr<const Eigen::MatrixXi> map_ptr_;
        std::array<int, 2> start_, goal_;
};
} // namespace plan

#endif // GLOBAL_PLAN_INTERFACE_HPP_