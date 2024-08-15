/*
 * @Author: Raiden49 
 * @Date: 2024-06-26 10:26:33 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-08-15 16:02:13
 */
#include "local_planner/offline_local_planner.hpp"

namespace local_planner 
{
/*基于三次多项式*/
/***
std::vector<std::vector<std::array<double, 2>>> OfflineLocalPlanner::PathGenerator(
        const double& heading, 
        const double& radius, const double& iter_angle,
        std::vector<std::array<double, 2>>& local_destination) {
    // Ax=B
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(4, 4);
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(4, 1);

    for (int angle = iter_angle; angle < 180; angle += iter_angle) {
        double radians = angle * M_PI / 180.0;
        double x = radius * cos(radians);
        double y = radius * sin(radians);

        local_destination.push_back({x, y});
    }

    double iter_num = 5;
    std::vector<std::vector<std::array<double, 2>>> local_solution;

    for (auto dest : local_destination) {

        double x = dest[0];
        double y = dest[1];

        if (x == 0) {
            std::vector<std::array<double, 2>> single_path;
            double iter_radius = y / iter_num;

            for (int j = 0; j < iter_num + 1; j++) {
                std::array<double, 2> point;
                double iter_y = j * iter_radius;
                double iter_x = 0;

                single_path.push_back({iter_x, iter_y});
            }
            local_solution.push_back(single_path);
            continue;
        }

        A << pow(0, 3), pow(0, 2), 0, 1,
             pow(x, 3), pow(x, 2), x, 1,
             3 * pow(0, 2), 2 * 0, 1, 0,
             3 * pow(x, 2), 2 * x, 1, 0; 
        B << 0, y, x > 0 ? 2 : -2, 0;
        auto factor_matrix = A.inverse() * B;

        std::vector<std::array<double, 2>> single_path;

        double iter_radius = x / iter_num;

        for (int j = 0; j < iter_num + 1; j++) {
            std::array<double, 2> point;
            double iter_x = j * iter_radius;
            double iter_y = factor_matrix(0, 0) * pow(iter_x, 3) + 
                            factor_matrix(1, 0) * pow(iter_x, 2) + 
                            factor_matrix(2, 0) * iter_x + 
                            factor_matrix(3, 0);

            single_path.push_back({iter_x, iter_y});
        }
        local_solution.push_back(single_path);
    }

    return local_solution;
}

std::vector<std::vector<std::array<double, 2>>> OfflineLocalPlanner::Process(
        const double& radius,
        const std::array<double, 2>& pos, 
        const std::array<double, 2>& pos_ahead, 
        std::vector<std::array<double, 2>>& destination) {
    
    std::vector<std::vector<std::array<double, 2>>> final_solution;
    std::vector<std::vector<std::vector<std::array<double, 2>>>> all_solution;

    destination.push_back({pos[0], pos[1]});
    double ahead_x = pos_ahead[0], ahead_y = pos_ahead[1];
    int current_size = destination.size(), last_size = 0;

    clock_t start_time = clock();

    // 分别生成三段路径
    double iter_angle = 15;
    for (int iter = 0; iter < 3; iter++) {
        std::vector<std::vector<std::array<double, 2>>> global_solution;
        for (int i = last_size; i < current_size; i++) {

            double x_0 = destination[i][0], y_0 = destination[i][1];
            double heading = atan2(y_0 - ahead_y, x_0 - ahead_x);
            std::vector<std::array<double, 2>> local_destination;

            auto local_solution = PathGenerator(
                    heading, radius, iter_angle * (iter + 1), local_destination);
            
            for (auto local_dest : local_destination) {
                auto global_point = Local2Global(local_dest, {x_0, y_0}, heading);
                destination.push_back(global_point);
            }
            for (auto local_solu : local_solution) {
                std::vector<std::array<double, 2>> single_path;
                for (auto local_point : local_solu) {
                    auto global_point = 
                            Local2Global(local_point, {x_0, y_0}, heading);
                    single_path.push_back(global_point);
                }
                global_solution.push_back(single_path);
            }
        }
        all_solution.push_back(global_solution);
        all_solution.push_back(global_solution);

        last_size = current_size;
        current_size = destination.size();
    }


    clock_t end_time = clock();

    // 合并路径
    int m1 = all_solution[0].size() / 1;
    int m2 = all_solution[1].size() / all_solution[0].size();
    int m3 = all_solution[2].size() / all_solution[1].size();
    for (int i = 0; i < all_solution[0].size(); i++) {
        for (int j = i * m2; j < i * m2 + m2; j++) {
            for (int k = j * m3; k < j * m3 + m3; k ++) {
                std::vector<std::array<double, 2>> single_path;
                for (auto point : all_solution[0][i]) {
                    single_path.push_back(point);
                }
                for (auto point : all_solution[1][j]) {
                    single_path.push_back(point);
                }
                for (auto point : all_solution[2][k]) {
                    single_path.push_back(point);
                }
                final_solution.push_back(single_path);
            }
        }
    }

    double run_time = (double)(end_time - start_time) * 1000 / CLOCKS_PER_SEC;
    std::cout << "Successfully get the local path in " 
              << run_time << " ms" << std::endl;

    return final_solution;
}
***/

std::vector<std::vector<Point3d>> OfflineLocalPlanner::PathGenerator(
        const double& radius, const double& iter_angle,
        std::vector<Point3d>& local_destination) {

    std::vector<std::vector<Point3d>> local_solution;
    std::vector<Point3d> path;
    path.push_back(Point3d());

    // 第二段每个点的路径数目是第一段的0.5倍，第三段则是第一段的1/3
    int iter_angle1 = iter_angle;
    int iter_angle2 = 2 * iter_angle, iter_angle3 = 3 * iter_angle;
    for (double angle1 = iter_angle1; angle1 < 180; angle1 += iter_angle1) {
        path.push_back(GetDestPoint(angle1, radius, path[0]));
        local_destination.push_back(path[1]);

        double heading_radians1 = 
                atan2(path[1].y - path[0].y, path[1].x - path[0].x);
        double heading_angle1 = heading_radians1 * 180 / M_PI;
        // 这里角度是根据旋转前后推到出来的，很简单
        for (double angle2 = -(90 - heading_angle1 - iter_angle2); 
                    angle2 < 90 + heading_angle1 - iter_angle2; 
                    angle2 += iter_angle2) {
            path.push_back(GetDestPoint(angle2, radius, path[1]));
            local_destination.push_back(path[2]);

            double heading_radians2 = 
                    atan2(path[2].y - path[1].y, path[2].x - path[1].x);
            double heading_angle2 = heading_radians2 * 180 / M_PI;
            for (double angle3 = -(90 - heading_angle2 - iter_angle3); 
                    angle3 < 90 + heading_angle2 - iter_angle3; 
                    angle3 += iter_angle3) {
                path.push_back(GetDestPoint(angle3, radius, path[2]));
                local_destination.push_back(path[3]); 
                local_solution.push_back(path);
                path.pop_back();
            }
            path.pop_back();
        }
        path.pop_back();
    }
    
    return local_solution;
}

std::vector<std::vector<Point3d>> OfflineLocalPlanner::Process(
        const double& radius, const Point3d& pos, 
        const Point3d& pos_ahead, std::vector<Point3d>& destination) {

    std::vector<Point3d> local_destination;
    std::vector<std::vector<Point3d>> optim_solution;
    std::vector<std::vector<Point3d>> global_solution;

    // 朝向就很简单的根据前后两个点计算反正切
    double heading = atan2(pos.y - pos_ahead.y, pos.x - pos_ahead.x);

    clock_t start_time = clock();

    double iter_angle = 10.;
    auto local_solution = PathGenerator(radius, iter_angle, local_destination);

    // 离线方法前面生成的路径中只包含了每一段的终点，需要Ｂ样条插值，得到完整路径
    for (auto& local_path : local_solution) {
        static auto optim_ptr = 
                std::make_shared<optimization::BSpline>(local_path, 20);
        optim_ptr->path_ptr_ = std::make_shared<std::vector<Point3d>>(local_path);
        optim_solution.push_back(optim_ptr->Process());
    }

    // 前面的计算都是在旋转后的坐标系下进行的，需要转换到世界坐标系
    for (auto& optim_path : optim_solution) {
        std::vector<Point3d> global_path;
        for (auto& path_point : optim_path) {
            auto global_point = Local2Global(path_point, pos, heading);
            global_path.push_back(global_point);
        }
        global_solution.push_back(global_path);
    }
    for (auto& local_dest : local_destination) {
        destination.push_back(Local2Global(local_dest, pos, heading));
    }

    clock_t end_time = clock();

    double run_time = (double)(end_time - start_time) * 1000 / CLOCKS_PER_SEC;
    std::cout << "Successfully get the local path in " 
              << run_time << " ms" << std::endl;

    return global_solution;

}
}