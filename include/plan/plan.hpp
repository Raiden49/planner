/*
 * @Author: Raiden49 
 * @Date: 2024-06-26 10:24:19 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-09-09 16:03:52
 */
#ifndef PLAN_HPP_
#define PLAN_HPP_

#include <time.h>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>

#include "global_planner/astar.hpp"
#include "global_planner/hybrid_astar.hpp"
#include "global_planner/rrt.hpp"
#include "global_planner/rrt_star.hpp"
#include "global_planner/jps.hpp"
#include "global_planner/lpa_star.hpp"
#include "optim/qp_optim.hpp"
#include "optim/bezier_optim.hpp"
#include "optim/b_spline_optim.hpp"
#include "local_planner/online_local_planner.hpp"
#include "local_planner/offline_local_planner.hpp"

namespace plan
{
/**
 * @brief 规划整体框架实现
 */
using Point3d = m_util::Point3d;
class Plan {
    public:
        Plan(ros::NodeHandle& nh) : nh_(nh) {
            ros::param::get("origin_x", origin_x_);
            ros::param::get("origin_y", origin_y_);
            ros::param::get("resolution", resolution_);
        }
        ~Plan() = default;

        void MapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg);
        void StartCallBack(
                const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
        void GoalCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void OdomCallBack(const nav_msgs::Odometry::ConstPtr& msg);

        /**
         * @brief 仿真底层控制实现，懒得调控制器，因此直接通过ｔｆ转换，移动到下一个路径点，看起来像是在跟踪路径
         * 
         * @param sim_index 路径点在整个路径中的序号
         * @param next_pos 下一个要到达的路径点
         * @param tf_stamped 用于发布ｔｆ转换信息的变量
         * @param path_ptr 指针，指向要跟踪的路径
         * @return true 可以成功跟踪
         * @return false 不可以跟踪
         */
        bool SimControllel(int& sim_index, const Point3d& next_point,
                geometry_msgs::TransformStamped& tf_stamped,
                const std::shared_ptr<std::vector<Point3d>>& path_ptr);

        /**
         * @brief 全局规划处理函数，规划出全局路径
         * 
         * @param global_path_msg 规划后的全局路径，用于可视化
         * @param global_world_path 返回的全局路径，世界坐标系下
         * @return true 成功规划出全局路径
         * @return false 全局路径规划失败
         */
        bool GlobalPathProcess(nav_msgs::Path& global_path_msg,
                               std::vector<Point3d>& global_world_path);
        /**
         * @brief 路径优化接口函数，对传入的路径通过指定方法做优化，主要是平滑，同时保持形状
         * 
         * @param optim_path_msg 优化后的路径信息，用于可视化
         * @param world_path 传入的待处理路径，世界坐标系
         * @param optim_method 指定优化方法，默认用二次规划
         * @return std::vector<std::array<double, 2>> 优化后路径
         */
        std::vector<Point3d> OptimPathProcess(nav_msgs::Path& optim_path_msg,
                                              const std::vector<Point3d>& world_path,
                                              const std::string& optim_method = "QP");
        /**
         * @brief 局部规划处理函数，整体流程是：生成一堆路径－＞选择得分最高的一条－＞优化－＞作为底层控制要跟踪的路径
         * 
         * @param sim_index 路径点在整个路径中的序号
         * @param ahead_pos 前一个路径点
         * @param current_pos 当前路径点，与前一个路径点大概求朝向角度
         * @param ref_path 参考路径，即全局路径
         * @param best_local_path 生成的所有路径中得分最高的
         * @return true 局部规划成功
         * @return false 局部规划不成功
         */
        bool LocalProcess(const int& sim_index, 
                          const Point3d& ahead_pos,
                          const Point3d& current_pos,
                          const std::vector<Point3d>& ref_path,
                          std::vector<Point3d>& best_local_path);

        void PublishVehiclePath(const ros::Publisher& vehicle_path_pub,
                                const std::vector<Point3d>& path,
                                const double& width, const double& length);
        // void PublishClearMarker(const ros::Publisher& clear_marker_pub);

        void DisplayToDebug(const std::shared_ptr<std::vector<Point3d>>& path_ptr);

        /**
         * @brief 主运行函数
         */
        void Process();

    private:
        ros::NodeHandle nh_;
        Point3d goal_, start_, current_pos_;
        //　地图信息
        int width_, height_;
        double resolution_, origin_x_, origin_y_;
        double current_radian_;
        //　用于判断是否接受到了新的起点或者终点的标志，接受到任何一个就意味着要重新做全局规划
        bool start_flag_ = false, goal_flag_ = false;
        Eigen::MatrixXi pgm_map_ = Eigen::Matrix<int, 1, 1>();
        ros::Subscriber map_sub_, odom_sub_, start_sub_, goal_sub_;

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

#endif // PLAN_HPP_