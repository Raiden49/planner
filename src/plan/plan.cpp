/*
 * @Author: Raiden49 
 * @Date: 2024-06-26 10:26:14 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-06-26 11:28:27
 */
#include "plan/plan.hpp"

namespace plan 
{
void Plan::MapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    ROS_INFO("Received a %d X %d map @ %.3f m/pix", 
            msg->info.width, msg->info.height, msg->info.resolution);

    width_ = msg->info.width; height_ = msg->info.height;
    resolution_ = msg->info.resolution;

    pgm_map_ = Eigen::MatrixXi::Zero(height_, width_);
    for (int row = 0; row < height_; row++) {
        for (int col = 0; col < width_; col++) {
            pgm_map_(row, col) = msg->data[col * width_ + row];
        }
    }
}

void Plan::StartCallBack(
        const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    ROS_INFO("Received the start point: %.2f, %.2f", 
            msg->pose.pose.position.x, msg->pose.pose.position.y);
    start_flag_ = true;
    start_[0] = msg->pose.pose.position.x;
    start_[1] = msg->pose.pose.position.y;
}

void Plan::GoalCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    ROS_INFO("Received the goal point: %.2f, %.2f", 
            msg->pose.position.x, msg->pose.position.y);
    goal_flag_ = true;
    goal_[0] = msg->pose.position.x;
    goal_[1] = msg->pose.position.y;
}

void Plan::OdomCallBack(const nav_msgs::Odometry::ConstPtr& msg) {
    // ROS_INFO("Current robot pos: %.2f, %.2f", 
    //         msg->pose.pose.position.x, msg->pose.pose.position.y);
    current_pos_[0] = msg->pose.pose.position.x;
    current_pos_[1] = msg->pose.pose.position.y; 
    current_radian_ = tf::getYaw(msg->pose.pose.orientation);
}

bool Plan::SimControllel(int& sim_index, const std::array<double, 2>& next_pos,
        geometry_msgs::TransformStamped& tf_stamped,
        const std::shared_ptr<std::vector<std::array<double, 2>>>& path_ptr) {

    static auto current_pos = next_pos;

    if (path_ptr != nullptr) {

        tf_stamped.header.stamp = ros::Time::now();
        tf_stamped.header.frame_id = "map";
        tf_stamped.child_frame_id = "new";

        tf_stamped.transform.translation.x = next_pos[0];
        tf_stamped.transform.translation.y = next_pos[1];
        double theta = atan2(next_pos[1] - current_pos[1], 
                             next_pos[0] - current_pos[0]);
        tf2::Quaternion qtn;
        if (sim_index == 0) {
            qtn.setRPY(0, 0, 0);
        }
        else {
            qtn.setRPY(0, 0, theta);
        }
        tf_stamped.transform.rotation.x = qtn.getX();
        tf_stamped.transform.rotation.y = qtn.getY();
        tf_stamped.transform.rotation.z = qtn.getZ();
        tf_stamped.transform.rotation.w = qtn.getW();

        // 找距离当前位置最近的路径点，遍历整条路径时间过长，
        // 方法为：如果当前位置往后１０个路径点距离均大于当前路径点，该路径点为最近点，
        // 否则较小的路径点为新的最近点开始新一轮迭代
        int count = 0, temp_index = sim_index;
        while (1) {
            count += 1;
            auto current_dis = 
                    m_util::EuclideanDis(next_pos[0], next_pos[1], 
                    path_ptr->at(temp_index)[0], 
                    path_ptr->at(temp_index)[1]);
            auto next_dis = temp_index + count < path_ptr->size() ? 
                    m_util::EuclideanDis(next_pos[0], next_pos[1],
                    path_ptr->at(temp_index + count)[0],
                    path_ptr->at(temp_index + count)[1]) : 
                    current_dis;
            if (current_dis > next_dis) {
                temp_index = temp_index + count;
                count = 0;
            }
            if (count >= 10) {
                sim_index = temp_index;
                break;
            }
        }

        current_pos = next_pos;
        return true;
    }

    return false;
}

bool Plan::GlobalPathProcess(nav_msgs::Path& global_path_msg,
        std::vector<std::array<int, 2>>& global_path, 
        std::vector<std::array<double, 2>>& global_world_path) {

    ROS_INFO("Start global plan");

    global_path_msg.poses.clear();
    global_path.clear();
    global_world_path.clear();

    auto start = World2Map(start_[0], start_[1]);
    auto goal = World2Map(goal_[0], goal_[1]);

    auto global_plan_ptr = 
            std::make_shared<global_planner::AStar>(pgm_map_, start, goal);

    // std::cout << "start point" << start[0] << ", " << start[1] << std::endl;
    // std::cout << "goal point" << goal[0] << ", " << goal[1] << std::endl;

    clock_t start_time = clock();
    if (!global_plan_ptr->GetPlan(global_path)) {
        ROS_ERROR("Error getting global path");
        return false;
    }
    clock_t end_time = clock();
    double run_time = (double)(end_time - start_time) * 1000 / CLOCKS_PER_SEC;
    std::cout << "Get the global path in " << run_time << " ms" << std::endl;

    for (auto iter = global_path.end() - 1; iter != global_path.begin(); iter--) {

        auto world_point = Map2World((*iter)[0], (*iter)[1]);
        global_world_path.push_back(world_point);
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map";
        pose.pose.position.x = world_point[0];
        pose.pose.position.y = world_point[1];
        pose.pose.orientation.w = 1.0;
        global_path_msg.poses.push_back(pose);
    }

    return true;

}

std::vector<std::array<double, 2>> Plan::OptimPathProcess(
        nav_msgs::Path& optim_path_msg,
        const std::vector<std::array<double, 2>>& world_path,
        const std::string& optim_method) {

    optim_path_msg.poses.clear();

    double lower_bound, upper_bound; 
    double weight_smooth, weight_length, weight_ref;
    ros::param::get("plan/QP/lower_bound", lower_bound);
    ros::param::get("plan/QP/upper_bound", upper_bound);
    ros::param::get("plan/QP/weight_smooth", weight_smooth);
    ros::param::get("plan/QP/weight_length", weight_length);
    ros::param::get("plan/QP/weight_ref", weight_ref);

    std::shared_ptr<optimization::OptimInterface> optim_ptr = 
            std::make_shared<optimization::QP>(lower_bound, upper_bound, 
            weight_smooth, weight_length, weight_ref, world_path);

    if (optim_method == "Bezier") {
        int num_samples;
        ros::param::get("plan/Bezier/num_samples", num_samples);
        optim_ptr = 
                std::make_shared<optimization::Bezier>(world_path, num_samples);
    }
    if (optim_method == "BSpline") {
        int num_samples;
        ros::param::get("plan/BSpline/num_samples", num_samples);
        optim_ptr = 
                std::make_shared<optimization::BSpline>(world_path, num_samples);
    }

    auto optim_path = optim_ptr->Process();

    for (auto& point : optim_path) {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map";
        pose.pose.position.x = point[0];
        pose.pose.position.y = point[1];
        pose.pose.orientation.w = 1.0;
        optim_path_msg.poses.push_back(pose);
    }

    return optim_path;
}

bool Plan::LocalProcess(const int& sim_index, 
                        const std::array<double, 2>& ahead_pos,
                        const std::array<double, 2>& current_pos,
                        const std::vector<std::array<double, 2>>& ref_path,
                        std::vector<std::array<double, 2>>& best_local_path) {

    static auto local_paths_pub = 
            nh_.advertise<visualization_msgs::MarkerArray>("local_paths", 1);
    static auto dest_pub_ = 
            nh_.advertise<visualization_msgs::Marker>("destionation", 1);
    static auto best_local_path_pub = 
            nh_.advertise<visualization_msgs::Marker>("best_local_path", 10);

    visualization_msgs::MarkerArray local_path_arr;
    auto destination_marker = m_util::CreateVisualMarker(
            1.0, {1., 0., 0.}, {0.01, 0.01}, "map", "point");
    auto best_path_marker = m_util::CreateVisualMarker(
            1.0, {1., 1., 0.}, {0.01, 0.01}, "map", "line");

    double radius = 0;
    std::string local_method;
    std::vector<std::array<double, 2>> destination;

    ros::param::get("plan/local_method", local_method);
    ros::param::get("plan/online/radius", radius);

    std::shared_ptr<local_planner::LocalPlannerInterface> local_planner = 
            std::make_shared<local_planner::OnlineLocalPlanner>(pgm_map_, 
                                                                ref_path);
    if (local_method == "offline") {
        ros::param::get("plan/offline/radius", radius);
        local_planner = 
                std::make_shared<local_planner::OfflineLocalPlanner>(pgm_map_, 
                                                                     ref_path);
    }
    
    local_planner->origin_x_ = origin_x_;
    local_planner->origin_y_ = origin_y_;
    local_planner->resolution_ = resolution_;
    
    auto local_path = local_planner->Process(radius, current_pos, 
                                             ahead_pos, destination);
    best_local_path = local_planner->GetBestPath(sim_index, local_path);
    
    // 生成的路径上有任何一点位于障碍物上的话，就删去整条路径
    for (int i = 0; i < local_path.size(); i++) {
        auto local_path_marker = m_util::CreateVisualMarker(
                0.3, {1., 0., 1.}, {0.005, 0.005}, "map", "line", i);
        for (int j = 0; j < local_path[i].size(); j++) {
            geometry_msgs::Point vtx;
            vtx.x = local_path[i][j][0];
            vtx.y = local_path[i][j][1];
            
            bool flag = false;
            auto map_point = World2Map(vtx.x, vtx.y);
            if (pgm_map_(map_point[0], map_point[1]) != 0) {
                break;
            }
            if (flag) {
                break;
            }

            local_path_marker.points.push_back(vtx);
        }
        local_path_arr.markers.push_back(local_path_marker);
    }
    local_paths_pub.publish(local_path_arr);

    for (int i = 0; i < destination.size(); i++) {
        geometry_msgs::Point vtx;
        vtx.x = destination[i][0];
        vtx.y = destination[i][1];
        destination_marker.points.push_back(vtx);
    }
    dest_pub_.publish(destination_marker);
    
    for (auto &point : best_local_path) {
        geometry_msgs::Point vtx;
        vtx.x = point[0];
        vtx.y = point[1];
        best_path_marker.points.push_back(vtx);
    }
    
    best_local_path_pub.publish(best_path_marker);

    return true;
    
}

void Plan::Process() {

    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom", 10, 
            std::bind(&Plan::OdomCallBack, this, std::placeholders::_1));
    map_sub_ = nh_.subscribe("/map", 1, &Plan::MapCallBack, this);
    start_sub_ = nh_.subscribe("/initialpose", 1, &Plan::StartCallBack, this);
    goal_sub_ = nh_.subscribe(
            "/move_base_simple/goal", 1, &Plan::GoalCallBack, this);

    auto global_path_pub = nh_.advertise<nav_msgs::Path>("global_path", 10);
    auto cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    auto optim_path_pub = nh_.advertise<nav_msgs::Path>("optim_global_path", 10);  
    auto optim_local_path_pub = 
            nh_.advertise<nav_msgs::Path>("optim_local_path", 10);  
    auto trajectory_pub = nh_.advertise<nav_msgs::Path>("trajectory", 10); 
    
    std::vector<std::array<int, 2>> global_path;
    std::vector<std::array<double, 2>> global_world_path;
    std::vector<std::array<double, 2>> best_local_path;
    std::shared_ptr<std::vector<std::array<double, 2>>> path_ptr = nullptr;
    std::shared_ptr<std::vector<std::array<double, 2>>> local_path_ptr = nullptr;
    nav_msgs::Path global_path_msg, optim_path_msg, optim_local_path_msg;
    nav_msgs::Path trajectory_msg;
    global_path_msg.header.frame_id = "map";
    optim_path_msg.header.frame_id = "map";
    optim_local_path_msg.header.frame_id = "map";
    trajectory_msg.header.frame_id = "map";

    tf2_ros::TransformBroadcaster sim_tf;
    geometry_msgs::TransformStamped tf_stamped;
    int sim_index = 0;
    
    ros::Rate rate(10);

    auto current_pos = start_;
    // 每一帧都进行控制，但每５帧才进行一次局部规划
    int frame_count = 0,local_frequency = 5;
    // 全局规划后的首次局部规划时，设置为true
    bool first_local_flag = true;
    std::string optim_method;
    ros::param::get("plan/optim_method", optim_method);
    while (ros::ok()) {

        // 起点或者终点更新，重新全局规划
        if (start_flag_ || goal_flag_) {

            start_flag_ = false; goal_flag_ = false; first_local_flag = true;
            sim_index = 0; frame_count = 0;
            trajectory_msg.poses.clear();

            // 全局规划规划出路径后，肯定是要进行优化的
            if(GlobalPathProcess(global_path_msg, global_path, 
                                 global_world_path)) {
                auto optim_path = OptimPathProcess(
                        optim_path_msg, global_world_path, optim_method);   
                path_ptr = std::make_shared<
                        std::vector<std::array<double, 2>>>(optim_path);   
                current_pos = path_ptr->at(0); 
            }
        }

        auto next_pos = current_pos;
        if (path_ptr != nullptr && sim_index < path_ptr->size() - 1) {
            // 前面5个路径点不进行局部规划，不影响，5个路径点可能就只是机器人长度
            // 因为仿真机器人朝向是根据前后两个位置直接计算的，这样比较方便
            if (first_local_flag) {
                first_local_flag = false;
                std::vector<std::array<double, 2>> temp_path;
                for (int i = 0; i < 2 * local_frequency; i++) {
                    temp_path.push_back(path_ptr->at(i));
                }
                local_path_ptr = std::make_shared<
                        std::vector<std::array<double, 2>>>(temp_path);
                frame_count = 0;
            }
            if (frame_count == local_frequency) {
                // 取当前局部规划的路径后五个路径点，以及下一次规划路径的前五个路径点，
                // 相当于为两次规划添加过渡，要不会有剧烈抖动
                std::vector<std::array<double, 2>> temp_path;
                for (int i = frame_count; i < 2 * local_frequency; i++) {
                    temp_path.push_back(local_path_ptr->at(i));
                }
                if (LocalProcess(sim_index, temp_path[temp_path.size() - 2],
                                 temp_path[temp_path.size() - 1], 
                                 *path_ptr, best_local_path)) {
                    for (int i = 0; i < frame_count; i++) {
                        temp_path.push_back(best_local_path[i]);
                    }
                    auto optim_local_path = OptimPathProcess(
                            optim_local_path_msg, temp_path);
                    local_path_ptr = std::make_shared<std::vector<
                            std::array<double, 2>>>(optim_local_path);
                    frame_count = 0;
                }
            }
            next_pos = local_path_ptr->at(frame_count);
        }

        // 控制仿真（在仿真中再仿真，确实有些．．．，但控制器也确实不好调）
        if (SimControllel(sim_index, next_pos, tf_stamped, path_ptr)) {
            sim_tf.sendTransform(tf_stamped);
        }

        current_pos = next_pos;
        frame_count += 1;
        
        // 轨迹可视化
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map";
        pose.pose.position.x = current_pos[0];
        pose.pose.position.y = current_pos[1];
        pose.pose.orientation.w = 1.0;
        if (path_ptr != nullptr) {
            trajectory_msg.poses.push_back(pose);
        }

        global_path_pub.publish(global_path_msg);
        optim_path_pub.publish(optim_path_msg);
        optim_local_path_pub.publish(optim_local_path_msg);
        trajectory_pub.publish(trajectory_msg);

        ros::spinOnce();
        rate.sleep();
    }
}
}