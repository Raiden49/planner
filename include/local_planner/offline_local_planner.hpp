/*
 * @Author: Raiden49 
 * @Date: 2024-06-26 10:25:02 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-08-15 15:40:12
 */
#ifndef OFFLINE_LOCAL_PLANNER_HPP_
#define OFFLINE_LOCAL_PLANNER_HPP_

#include "local_planner/local_planner_interface.hpp"

namespace local_planner
{
/**
 * @brief 离线路径生成方法，这里本来是应该设置为直接读取提前生成好的路径，所以是离线方法，
 * 但是，离线方法用三次样条生成几百条路径的速度居然比在线方法用Eiegn计算五次多项式生成几十条速度要快，
 * 所以我懒得离线了，用三次样条生成同样的路径比用Eigen计算三次多项式快好几倍，感觉Eigen计算有点慢了是否
 */
class OfflineLocalPlanner : public LocalPlannerInterface {
    public:
        OfflineLocalPlanner(const Eigen::MatrixXi& map, 
                            const std::vector<Point3d>& ref_path) :
                            LocalPlannerInterface(map, ref_path) {};
        ~OfflineLocalPlanner() = default;
        /**
         * @brief 离线方法中的路径生成函数，但这里主要只生成终点,然后在主函数中进行三次样条插值
         * 
         * @param radius 每段路径的半径长度,一共生成三段，每段路径半径一致
         * @param iter_angle 根据角度范围生成路径的，所以有迭代角度大小，越小路径越多
         * @return std::vector<std::vector<std::array<double, 2>>> 生成的路径集合
         */
        std::vector<std::vector<Point3d>> PathGenerator(
                const double& radius, const double& iter_angle,
                std::vector<Point3d>& local_destination);
        std::vector<std::vector<Point3d>> Process(
                const double& radius, const Point3d& pos, 
                const Point3d& pos_ahead, std::vector<Point3d>& destination) override;
    public:
        inline Point3d GetDestPoint(const double& angle, const double& radius,
                                    const Point3d& origin_pos) {
            double radians = angle * M_PI / 180.0;
            double x = radius * cos(radians) + origin_pos.x;
            double y = radius * sin(radians) + origin_pos.y;

            return Point3d(x, y, 0);
        }
};
}

#endif // OFFLINE_LOCAL_PLANNER_HPP_