/*
 * @Author: Raiden49 
 * @Date: 2024-06-26 10:24:38 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-08-15 15:17:31
 */
#ifndef GLOBAL_PLAN_INTERFACE_HPP_
#define GLOBAL_PLAN_INTERFACE_HPP_

#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <memory>

#include "m_util.hpp"

namespace global_planner
{
/**
 * @brief 全局规划父类，纯接口类，无函数实现
 */
using Point3d = m_util::Point3d;
class GlobalPlannerInterface {
    public:
        GlobalPlannerInterface(const Eigen::MatrixXi& map,
                               const std::array<int, 2>& start, 
                               const std::array<int, 2>& goal) : 
                               start_(start), goal_(goal) {
            map_ptr_ = std::make_shared<Eigen::MatrixXi>(map);
        }
        virtual ~GlobalPlannerInterface() {};
        virtual bool GetPlan(std::vector<Point3d>& path) = 0;
    public:
        double origin_x_, origin_y_, resolution_;
        std::shared_ptr<const Eigen::MatrixXi> map_ptr_;
        std::array<int, 2> start_, goal_;
    public:
        inline std::array<int, 2> World2Map(const double& x, const double& y) {
            int map_x = (x - origin_x_) / resolution_;
            int map_y = (y - origin_y_) / resolution_;
            return {map_x, map_y};
        }
        inline std::array<double, 2> Map2World(const int& x, const int& y) {
            double world_x = x * resolution_ + origin_x_;
            double world_y = y * resolution_ + origin_y_;
            return {world_x, world_y};
        }
};
} // namespace plan

#endif // GLOBAL_PLAN_INTERFACE_HPP_