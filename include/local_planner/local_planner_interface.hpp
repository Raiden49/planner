/*
 * @Author: Raiden49 
 * @Date: 2024-06-26 10:24:59 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-08-15 15:34:12
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
using Point3d = m_util::Point3d;
class LocalPlannerInterface {
    public:
        LocalPlannerInterface(const Eigen::MatrixXi& map, 
                              const std::vector<Point3d>& ref_path) {
            map_ptr_ = std::make_shared<Eigen::MatrixXi>(map);
            path_ptr_ = std::make_shared<std::vector<Point3d>>(ref_path);
            map_tool_->set_map_ptr_(map);
        }
        virtual ~LocalPlannerInterface() {};

        /**
         * @brief 旋转后的坐标系的点变换到世界坐标系下
         */
        Point3d Local2Global(const Point3d& local_point, const Point3d& shift_point,
                             const double& theta);
        /**
         * @brief 世界坐标系下点变换到旋转后坐标系下
         */
        Point3d Global2Local(const Point3d& global_point, const Point3d& shift_point,
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
        virtual std::vector<std::vector<Point3d>> Process(
                const double& radius, const Point3d& pos, 
                const Point3d& pos_ahead, std::vector<Point3d>& destination) = 0;
        /**
         * @brief 从所有生成的路径中选取得分最高的一个，根据距离障碍物距离和参考线距离打分
         * 
         * @return std::vector<std::array<double, 2>> 得分最高的路径
         */
        std::vector<Point3d> GetBestPath(
                int current_index, const std::vector<std::vector<Point3d>>& paths);

    public:
        double resolution_, origin_x_, origin_y_;
        std::shared_ptr<const std::vector<Point3d>> path_ptr_;
        std::shared_ptr<const Eigen::MatrixXi> map_ptr_;
        std::shared_ptr<m_util::MapInfoTool> 
                map_tool_ = std::make_shared<m_util::MapInfoTool>();
};
}

#endif // LOCAL_PLANNER_INTERFACE_HPP_