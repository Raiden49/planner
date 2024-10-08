/*
 * @Author: Raiden49 
 * @Date: 2024-07-19 16:47:59 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-08-16 16:00:00
 */
#include "global_planner/hybrid_astar.hpp"

namespace global_planner
{
void HybridAstar::SetVehicleShape(const double& length, const double& width, 
                                  const double& rear_axle_dis) {
    // 上下左右四个顶点
    vehicle_shape_.resize(8);
    vehicle_shape_.block<2, 1>(0, 0) = Eigen::Vector2d(-rear_axle_dis, 
                                                       width / 2);
    vehicle_shape_.block<2, 1>(2, 0) = Eigen::Vector2d(length - rear_axle_dis, 
                                                       width / 2);
    vehicle_shape_.block<2, 1>(4, 0) = Eigen::Vector2d(length - rear_axle_dis, 
                                                       -width / 2);
    vehicle_shape_.block<2, 1>(6, 0) = Eigen::Vector2d(-rear_axle_dis, 
                                                       -width / 2);
    double step_size = move_step_size_;
    auto length_discrete_num = (int) (length / move_step_size_);
    auto width_discrete_num =  (int) (width / move_step_size_);
    double all_point_num = (length_discrete_num + width_discrete_num) * 2;
    vehicle_shape_discrete_.resize(2, all_point_num);

    Eigen::Vector2d edge_0_normalized = vehicle_shape_.block<2, 1>(2, 0) - 
                                        vehicle_shape_.block<2, 1>(0, 0);
    edge_0_normalized = edge_0_normalized.normalized();
    for (int i = 0; i < length_discrete_num; i++) {
        vehicle_shape_discrete_.block<2, 1>(0, i + length_discrete_num) = 
                vehicle_shape_.block<2, 1>(4, 0) - edge_0_normalized * i * step_size;
        vehicle_shape_discrete_.block<2, 1>(0, i) = 
                vehicle_shape_.block<2, 1>(0, 0) + edge_0_normalized * i * step_size;
    }

    Eigen::Vector2d edge_1_normalized = vehicle_shape_.block<2, 1>(4, 0) - 
                                        vehicle_shape_.block<2, 1>(2, 0);
    edge_1_normalized = edge_0_normalized.normalized();
    for (int i = 0; i < width_discrete_num; i++) {
        vehicle_shape_discrete_.block<2, 1>(0, 2 * length_discrete_num + i) =
                vehicle_shape_.block<2, 1>(2, 0) + edge_1_normalized * i * step_size;
        vehicle_shape_discrete_.block<2, 1>(0, 2 * length_discrete_num + i + width_discrete_num) = 
                vehicle_shape_.block<2, 1>(6, 0) - edge_1_normalized * i * step_size;
    }
}
void HybridAstar::InitConfig() {

    double turning_radius = wheel_base_ / tan(steering_radian_);
    turning_radius = 1;
    std::cout << "turning radius:" << turning_radius << std::endl;
    rs_ptr_ = std::make_shared<ompl::base::ReedsSheppStateSpace>(turning_radius);

    // copyed
    SetVehicleShape(0.5, 0.5, 0.25);

    node_ptr_map.resize(map_ptr_->rows());
    for (int row = 0; row < map_ptr_->rows(); row++) {
        node_ptr_map[row].resize(map_ptr_->cols());
        for (int col = 0; col < map_ptr_->cols(); col++) {
            node_ptr_map[row][col].resize(360);
        }
    }
}
double HybridAstar::ComputeH(const HybridNode& current_node, 
                             const HybridNode& end_node) {
    double h_cost = m_util::EuclideanDis(current_node.world_point_.x,
                                    current_node.world_point_.y,
                                    end_node.world_point_.x,
                                    end_node.world_point_.y);
    if (h_cost <  shot_dis_) {
        ompl::base::ScopedState<ompl::base::SE2StateSpace> rs_start(rs_ptr_);
        ompl::base::ScopedState<ompl::base::SE2StateSpace> rs_goal(rs_ptr_);
        rs_start->setX(current_node.world_point_.x);
        rs_start->setY(current_node.world_point_.y);
        rs_start->setYaw(current_node.world_point_.yaw);
        rs_goal->setX(end_node.world_point_.x);
        rs_goal->setY(end_node.world_point_.y);
        rs_goal->setYaw(end_node.world_point_.yaw);
        h_cost = rs_ptr_->distance(rs_start.get(), rs_goal.get());
    }
    return h_cost;
}
double HybridAstar::ComputeG(const HybridNode& current_node,
                             const HybridNode& neighbor_node) {
    double g_cost = 0;
    if (neighbor_node.direction_ == HybridNode::FORWARD) {
        if (neighbor_node.steering_grade_ != current_node.steering_grade_) {
            if (neighbor_node.steering_grade_ == 0) {
                g_cost = segment_length_ * steering_change_penalty_;
            }
            else {
                g_cost = segment_length_ * steering_change_penalty_ * 
                         steering_penalty_;
            }
        }
        else {
            if (neighbor_node.steering_grade_ == 0) {
                g_cost = segment_length_;
            }
            else {
                g_cost = segment_length_ * steering_penalty_;
            }
        }
    }
    else {
        if (neighbor_node.steering_grade_ != current_node.steering_grade_) {
            if (neighbor_node.steering_grade_ == 0) {
                g_cost = segment_length_ * steering_change_penalty_ * 
                         reversing_penalty_;
            }
            else {
                g_cost = segment_length_ * steering_change_penalty_ * 
                         steering_penalty_ * reversing_penalty_;
            }
        }
        else {
            if (neighbor_node.steering_grade_ == 0) {
                g_cost = segment_length_ * reversing_penalty_;
            }
            else {
                g_cost = segment_length_ * steering_penalty_ * reversing_penalty_;
            }
        }
    }

    return g_cost;
}
bool HybridAstar::CheckCollision(const Point3d& point) {
    Eigen::Matrix2d rotate_matrix;
    rotate_matrix << cos(point.yaw), -sin(point.yaw),
                     sin(point.yaw), cos(point.yaw);
    
    std::vector<m_util::Point2t<int>> map_point_vec;
    for (int i = 0; i < 4; i++) {
        auto temp_pos = rotate_matrix * vehicle_shape_.block<2, 1>(i * 2, 0) + 
                        Eigen::Vector2d(point.x, point.y);
        map_point_vec.push_back(map_tool_->World2Map(temp_pos(0, 0), temp_pos(1, 0)));
    }
    int x_1 = map_point_vec[0].x, y_1 = map_point_vec[0].y;
    int x_2 = map_point_vec[1].x, y_2 = map_point_vec[1].y;
    int x_3 = map_point_vec[2].x, y_3 = map_point_vec[2].y;
    int x_4 = map_point_vec[3].x, y_4 = map_point_vec[3].y;
    if (map_tool_->IsLineAvailable(x_1, y_1, x_2, y_2) && 
        map_tool_->IsLineAvailable(x_2, y_2, x_3, y_3) &&
        map_tool_->IsLineAvailable(x_3, y_3, x_4, y_4) && 
        map_tool_->IsLineAvailable(x_1, y_1, x_4, y_4)) {
        return true;
    }
    return false;
}
bool HybridAstar::AnalyticExpansions(HybridNode& current_node, 
                                     HybridNode& end_node) {
    ompl::base::ScopedState<ompl::base::SE2StateSpace> rs_start(rs_ptr_);
    ompl::base::ScopedState<ompl::base::SE2StateSpace> rs_goal(rs_ptr_);
    rs_start->setX(current_node.world_point_.x);
    rs_start->setY(current_node.world_point_.y);
    rs_start->setYaw(current_node.world_point_.yaw);
    rs_goal->setX(end_node.world_point_.x);
    rs_goal->setY(end_node.world_point_.y);
    rs_goal->setYaw(end_node.world_point_.yaw);

    double rs_path_length = rs_ptr_->distance(rs_start.get(), rs_goal.get());
    int num_segment = rs_path_length / move_step_size_;
    std::vector<Point3d> rs_path;
    for (int i = 0; i < num_segment; i++){
        ompl::base::State* state = rs_ptr_->allocState();
        rs_ptr_->interpolate(rs_start.get(), rs_goal.get(), 
                             (double)i / num_segment, state);
        auto* se2state = state->as<ompl::base::SE2StateSpace::StateType>();
        Point3d temp_node(se2state->getX(), se2state->getY(), se2state->getYaw());
        rs_path.push_back(temp_node);
    }
    for (const auto& rs_point : rs_path) {
        auto point = map_tool_->World2Map(rs_point.x, rs_point.y);
        if (!map_tool_->InBoundary(point.x, point.y) || !CheckCollision(rs_point)) {
            return false;
        }
    }
    end_node.intermediate_point_ = rs_path;
    end_node.intermediate_point_.erase(end_node.intermediate_point_.begin());
    end_node.parent_ = std::make_shared<HybridNode>(current_node);
    return true;
}
inline double HybridAstar::Mod2Pi(const double& theta) {
    double result = fmod(theta, 2 * M_PI);
    if (result < -M_PI) {
        result += 2 * M_PI;
    } else if (result > M_PI) {
        result -= 2 * M_PI;
    }

    return result;
}
inline void HybridAstar::MotionModel(const double& step_size, const double& phi,
                                     double& x, double& y, double& theta) {
    x = x + step_size * cos(theta);
    y = y + step_size * sin(theta);
    // theta = Mod2Pi(theta + step_size / wheel_base_ * tan(phi));
    theta = Mod2Pi(theta + 0.3 * tan(phi));
}
std::vector<HybridNode> HybridAstar::GetNeighborNodes(const HybridNode& current_node) {
    std::vector<HybridNode> neighbor_nodes;
    for (int i = -steering_discrete_num; i <= steering_discrete_num; i++) {
        std::vector<Point3d> intermediate_points;
        double x = current_node.world_point_.x;
        double y = current_node.world_point_.y;
        double theta = current_node.world_point_.yaw;
        const double phi = i * steering_radian_step_size_;
        bool has_obstacle = false;
        
        // forward
        for (int j = 1; j <= segment_length_discrete_num_; j++) {
            MotionModel(move_step_size_, phi, x, y, theta);
            intermediate_points.emplace_back(Point3d(x, y, theta));
            if (!CheckCollision(Point3d(x, y, theta))) {
                has_obstacle = true;
                break;
            }
        }
        auto point = intermediate_points.back();
        auto map_point = map_tool_->World2Map(point.x, point.y);
        if (map_tool_->InBoundary(map_point.x, map_point.y) && !has_obstacle) {
            auto neighbor_node_ptr = std::make_shared<HybridNode>(map_point.x,
                                                                  map_point.y,
                                                                  point);
            neighbor_node_ptr->intermediate_point_ = intermediate_points;
            neighbor_node_ptr->steering_grade_ = i;
            neighbor_node_ptr->direction_ = HybridNode::FORWARD;
            neighbor_nodes.push_back(*neighbor_node_ptr);
        }

        // backward 
        has_obstacle = false;
        intermediate_points.clear();
        x = current_node.world_point_.x; 
        y = current_node.world_point_.y;
        theta = current_node.world_point_.yaw;
        for (int j = 1; j <= segment_length_discrete_num_; j++) {
            MotionModel(-move_step_size_, phi, x, y, theta);
            intermediate_points.emplace_back(Point3d(x, y, theta));
            if (!CheckCollision(Point3d(x, y, theta))) {
                has_obstacle = true;
                break;
            }
        }
        point = intermediate_points.back();
        map_point = map_tool_->World2Map(point.x, point.y);
        if (map_tool_->InBoundary(map_point.x, map_point.y) && !has_obstacle) {
            auto neighbor_node_ptr = std::make_shared<HybridNode>(map_point.x,
                                                                  map_point.y,
                                                                  point);
            neighbor_node_ptr->intermediate_point_ = intermediate_points;
            neighbor_node_ptr->steering_grade_ = i;
            neighbor_node_ptr->direction_ = HybridNode::BACKWARD;
            neighbor_nodes.push_back(*neighbor_node_ptr);
        }
    }
    return neighbor_nodes;
}
inline std::vector<int> HybridAstar::GetNodePtrMapIndex(const Point3d& point) {
    std::vector<int> result;
    auto map_point = map_tool_->World2Map(point.x, point.y);
    int angle = point.yaw * 180 / M_PI + 180;
    result.push_back(map_point.x);
    result.push_back(map_point.y);
    result.push_back(angle);
    
    return result;
}
bool HybridAstar::Search(const Point3d& start_point, const Point3d& goal_point) {
    auto start_map_point = map_tool_->World2Map(start_point.x, start_point.y);
    auto goal_map_point = map_tool_->World2Map(goal_point.x, goal_point.y);

    std::shared_ptr<global_planner::HybridNode> start_ptr = 
            std::make_shared<global_planner::HybridNode>(start_map_point.x, 
                                                         start_map_point.y, 
                                                         start_point);
    start_ptr->direction_ = HybridNode::NO;
    start_ptr->steering_grade_ = 0;
    start_ptr->status_ = HybridNode::IN_OPENSET;
    start_ptr->g_cost_ = 0.0;
    start_ptr->intermediate_point_.push_back(start_ptr->world_point_);

    std::shared_ptr<global_planner::HybridNode> goal_ptr = 
            std::make_shared<global_planner::HybridNode>(goal_map_point.x, 
                                                         goal_map_point.y, 
                                                         goal_point);
    goal_ptr->direction_ = HybridNode::NO;
    goal_ptr->steering_grade_ = 0;                                
    
    start_ptr->h_cost_ = ComputeH(*start_ptr, *goal_ptr);
    open_set_.clear();
    open_set_.insert(std::make_pair(start_ptr->get_fcost(), start_ptr));

    goal_ptr_ = goal_ptr;

    std::shared_ptr<HybridNode> current_ptr;
    int iter_count = 0;
    while (!open_set_.empty()) {
        current_ptr = open_set_.begin()->second;
        current_ptr->status_ = HybridNode::IN_CLOSESET;
        open_set_.erase(open_set_.begin());

        auto cur_dis = m_util::EuclideanDis(current_ptr->world_point_.x,
                                            current_ptr->world_point_.y,
                                            goal_ptr->world_point_.x,
                                            goal_ptr->world_point_.y);
        if (cur_dis <= shot_dis_) {
            if (AnalyticExpansions(*current_ptr, *goal_ptr)) {
                ROS_INFO("The Hybrid A star has found a path!!!");
                return true;
            }
        }
        auto neighbor_nodes = GetNeighborNodes(*current_ptr);
        for (auto& neighbor_node : neighbor_nodes) {
            auto neighbor_edge_cost = ComputeG(*current_ptr, neighbor_node);
            auto h_cost = ComputeH(*current_ptr, *goal_ptr);
            auto index_vec = GetNodePtrMapIndex(neighbor_node.world_point_);
    
            auto node_ptr = node_ptr_map[index_vec[0]][index_vec[1]][index_vec[2]];
            if (node_ptr == nullptr) {
                neighbor_node.g_cost_ = current_ptr->g_cost_ + neighbor_edge_cost;
                neighbor_node.parent_ = current_ptr;
                neighbor_node.status_ = HybridNode::IN_OPENSET;
                neighbor_node.h_cost_ = h_cost;
                // open_set_.insert(std::make_pair(neighbor_node.h_cost_, 
                //                  std::make_shared<HybridNode>(neighbor_node)));
                open_set_.insert(std::make_pair(neighbor_node.get_fcost(), 
                                 std::make_shared<HybridNode>(neighbor_node)));
                node_ptr_map[index_vec[0]][index_vec[1]][index_vec[2]] = 
                        std::make_shared<HybridNode>(neighbor_node);
            }
            else if (node_ptr->status_ == HybridNode::IN_OPENSET) {
                double current_g_cost = current_ptr->g_cost_ + neighbor_edge_cost;
                if (node_ptr->g_cost_ > current_g_cost) {
                    neighbor_node.g_cost_ = current_g_cost;
                    neighbor_node.h_cost_ = h_cost;
                    neighbor_node.parent_ = current_ptr;
                    neighbor_node.status_ = HybridNode::IN_OPENSET;
                }
            }
            else if (node_ptr->status_ == HybridNode::IN_CLOSESET) {
                continue;
            }
        }
        iter_count++;
        if (iter_count > 50000) {
            ROS_WARN("the number of iterations is too high, search failed!!!");
            return false;
        }
    }
    
    return false;
}
bool HybridAstar::GetPlan(std::vector<Point3d>& path) {
    InitConfig();
    Point3d start_pose(map_tool_->Map2World(start_[0], start_[1])); 
    start_pose.yaw = start_yaw_;
    Point3d goal_pose(map_tool_->Map2World(goal_[0], goal_[1])); 
    goal_pose.yaw = goal_yaw_;
    if (Search(start_pose, goal_pose)) {
        std::vector<std::shared_ptr<HybridNode>> node_ptr_vec;
        auto node_ptr = goal_ptr_;
        while (node_ptr != nullptr) {
            node_ptr_vec.emplace_back(node_ptr);
            node_ptr = node_ptr->parent_;
        }
        std::reverse(node_ptr_vec.begin(), node_ptr_vec.end());
        for (auto& node_ptr : node_ptr_vec) {
            for (auto& point : node_ptr->intermediate_point_) {
                path.push_back(point);
            }
        }
        return true;
    }
    return false;
}
}