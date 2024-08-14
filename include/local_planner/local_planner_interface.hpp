/*
 * @Author: Raiden49 
 * @Date: 2024-06-26 10:24:59 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-06-26 11:06:35
 */
#ifndef LOCAL_PLANNER_INTERFACE_HPP_
#define LOCAL_PLANNER_INTERFACE_HPP_

#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <time.h>
#include <array>
#include <memory>

#include "m_util.hpp"
#include "optim/b_spline_optim.hpp"

namespace local_planner 
{
/**
 * @brief 局部规划父类函数，不包含生成路径的实现
 */
class LocalPlannerInterface {
    public:
        LocalPlannerInterface(
                const Eigen::MatrixXi& map, 
                const std::vector<std::array<double, 2>>& ref_path) {
            map_ptr_ = std::make_shared<Eigen::MatrixXi>(map);
            path_ptr_ = std::make_shared<
                    std::vector<std::array<double, 2>>>(ref_path);
        }
        virtual ~LocalPlannerInterface() {};

        /**
         * @brief 旋转后的坐标系的点变换到世界坐标系下
         */
        std::array<double, 2> Local2Global(
                const std::array<double, 2>& local_point, 
                const std::array<double, 2>& shift_point,
                const double& theta);
        /**
         * @brief 世界坐标系下点变换到旋转后坐标系下
         */
        std::array<double, 2> Global2Local(
                const std::array<double, 2>& global_point, 
                const std::array<double, 2>& shift_point,
                const double& theta);
        /**
         * @brief 生成路径的主运行函数，纯虚函数
         * 
         * @param radius 半径，即生成路径的半径大小
         * @param pos 当前位置点
         * @param pos_ahead 前一个路径点
         * @param destination 生成的路径终点信息
         * @return std::vector<std::vector<std::array<double, 2>>> 所有生成的路径
         */
        virtual std::vector<std::vector<std::array<double, 2>>> Process(
                const double& radius,
                const std::array<double, 2>& pos, 
                const std::array<double, 2>& pos_ahead, 
                std::vector<std::array<double, 2>>& destination) = 0;
        /**
         * @brief 从所有生成的路径中选取得分最高的一个，根据距离障碍物距离和参考线距离打分
         * 
         * @return std::vector<std::array<double, 2>> 得分最高的路径
         */
        std::vector<std::array<double, 2>> GetBestPath(int current_index,
                const std::vector<std::vector<std::array<double, 2>>>& paths);

    public:
        double resolution_, origin_x_, origin_y_;
        std::shared_ptr<const std::vector<std::array<double, 2>>> path_ptr_;
        std::shared_ptr<const Eigen::MatrixXi> map_ptr_;

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
}

#endif // LOCAL_PLANNER_INTERFACE_HPP_