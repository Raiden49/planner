/*
 * @Author: Raiden49 
 * @Date: 2024-07-19 16:37:59 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-08-15 15:19:23
 */
#ifndef HYBRID_ASTAR_HPP_
#define HYBRID_ASTAR_HPP_

#include "global_planner/global_planner_interface.hpp"
#include "global_planner/hybrid_astar_type.hpp"
#include <ompl/base/State.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>

namespace global_planner
{
class HybridAstar : public GlobalPlannerInterface {
    public:
        HybridAstar(const double& steering_angle, 
                    const int& steering_angle_discrete_num,
                    const double& wheel_base, 
                    const double& segment_length, 
                    const int& segment_length_discrete_num,
                    const double& steering_penalty,
                    const double& steering_change_penalty,
                    const double& reversing_penalty, 
                    const double shot_dis,
                    const double& start_yaw, const double& goal_yaw,
                    const Eigen::MatrixXi& map, const std::array<int, 2>& start, 
                    const std::array<int, 2>& goal) : 
                    GlobalPlannerInterface(map, start, goal), 
                    start_yaw_(start_yaw), goal_yaw_(goal_yaw) {
            this->wheel_base_ = wheel_base;
            this->segment_length_ = segment_length;
            this->steering_radian_ = steering_angle * M_PI / 180.0;
            this->steering_discrete_num = steering_angle_discrete_num;
            this->steering_radian_step_size_ = steering_radian_ / steering_discrete_num;
            this->move_step_size_ = segment_length / segment_length_discrete_num;
            this->segment_length_discrete_num_ = segment_length_discrete_num;
            this->steering_penalty_ = steering_penalty;
            this->steering_change_penalty_ = steering_change_penalty;
            this->reversing_penalty_ = reversing_penalty;
            this->shot_dis_ = shot_dis;
            this->tie_breaker_ = 1.0 + 1e-3;
        };
        ~HybridAstar() override = default;

        void SetVehicleShape(const double& length, const double& width, 
                             const double& rear_axle_dis);
        void InitConfig();
        double ComputeH(const HybridNode& current_node, 
                        const HybridNode& end_node);
        double ComputeG(const HybridNode& current_node,
                        const HybridNode& neighbor_node);
        bool CheckCollision(const Point3d& point);
        bool AnalyticExpansions(HybridNode& current_node, HybridNode& end_node);
        inline void MotionModel(const double& step_size, const double& phi,
                                double& x, double& y, double& theta);
        inline double Mod2Pi(const double& theta);
        inline std::vector<int> GetNodePtrMapIndex(const Point3d& point);
        std::vector<HybridNode> GetNeighborNodes(const HybridNode& current_node);
        bool Search(const Point3d& start_point, const Point3d& goal_point);
        bool GetPlan(std::vector<Point3d>& path) override;

    private:
        // HybridNode ***node_state_map_;
        std::vector<std::vector<std::vector<std::shared_ptr<HybridNode>>>> node_ptr_map;
        double start_yaw_, goal_yaw_;
        std::shared_ptr<HybridNode> goal_ptr_ = nullptr;
        std::multimap<double, std::shared_ptr<HybridNode>> open_set_, close_set_;
        // ReedSheep config
        std::shared_ptr<ompl::base::ReedsSheppStateSpace> rs_ptr_ = 
                std::make_shared<ompl::base::ReedsSheppStateSpace>(1.0);
    private:
        double wheel_base_;//车辆轴距
        double segment_length_;//路径段长度
        int segment_length_discrete_num_;//路径段长度离散数目，可以理解为插值点
        double steering_radian_;//转向角弧度
        int steering_discrete_num;//转向角的离散数目
        double steering_radian_step_size_;//每个离散转向角弧度大小
        double move_step_size_;//每个离散路径段长度
        double steering_penalty_;//转向惩罚系数
        double steering_change_penalty_;//转向变换惩罚系数
        double reversing_penalty_;//倒车惩罚系数
        double shot_dis_;//直接距离
        double tie_breaker_;
    private:
        Eigen::VectorXd vehicle_shape_;
        Eigen::MatrixXd vehicle_shape_discrete_;
        
};
}

#endif // HYBRID_ASTAR_HPP_