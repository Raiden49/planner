/*
 * @Author: Raiden49 
 * @Date: 2024-06-26 10:26:36 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-06-26 11:49:02
 */
#include "local_planner/online_local_planner.hpp"

namespace local_planner 
{
std::vector<std::vector<std::array<double, 2>>> OnlineLocalPlanner::Process(
        const double& radius,
        const std::array<double, 2>& pos, 
        const std::array<double, 2>& pos_ahead, 
        std::vector<std::array<double, 2>>& destination) {
    
    std::vector<std::vector<std::array<double, 2>>> local_solution;
    std::vector<std::vector<std::array<double, 2>>> final_solution;

    auto local_destinaton = destination;
    destination.push_back({pos[0], pos[1]});
    double x_0 = pos[0], y_0 = pos[1];
    double ahead_x = pos_ahead[0], ahead_y = pos_ahead[1];
    int current_size = destination.size(), last_size = 0;

    clock_t start_time = clock();

    // Ax=B，就是一个五次多项式，满足６个等式方程，求解各个系数
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(6, 6);
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(6, 1);

    double iter_length = radius / 30, y = radius;
    for (double x = -y; x <= y; x += iter_length) {
        local_destinaton.push_back({x, y});
    }

    double iter_num = 10;
    double heading = atan2(y_0 - ahead_y, x_0 - ahead_x);

    for (auto dest : local_destinaton) {

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

        A << pow(0, 5), pow(0, 4), pow(0, 3), pow(0, 2), pow(0, 1), 1,
             5 * pow(0, 4), 4 * pow(0, 3), 3 * pow(0, 2), 2 * pow(0, 1), 1, 0,
             20 * pow(0, 3), 12 * pow(0, 2), 6 * pow(0, 1), 2, 0, 0,
             pow(x, 5), pow(x, 4), pow(x, 3), pow(x, 2), pow(x, 1), 1,
             5 * pow(x, 4), 4 * pow(x, 3), 3 * pow(x, 2), 2 * pow(x, 1), 1, 0,
             20 * pow(x, 3), 12 * pow(x, 2), 6 * pow(x, 1), 2, 0, 0;
        // 这里３是因为，９０度的正切是无穷的，用３代替就够了，也不用特别准确
        B << 0, x > 0 ? 3 : -3, 0, y, x > 0 ? 3 : -3, 0;
        auto factor_matrix = A.inverse() * B;

        std::vector<std::array<double, 2>> single_path;

        double iter_radius = x / iter_num;

        // 根据计算出来的五次多项式系数，计算多个点合成一条路径
        for (int j = 0; j < iter_num + 1; j++) {
            std::array<double, 2> point;
            double iter_x = j * iter_radius;
            double iter_y = factor_matrix(0, 0) * pow(iter_x, 5) + 
                            factor_matrix(1, 0) * pow(iter_x, 4) + 
                            factor_matrix(2, 0) * pow(iter_x, 3) + 
                            factor_matrix(3, 0) * pow(iter_x, 2) + 
                            factor_matrix(4, 0) * pow(iter_x, 1) + 
                            factor_matrix(5, 0);

            single_path.push_back({iter_x, iter_y});
        }
        local_solution.push_back(single_path);
    }

    for (auto& local_solu : local_solution) {
        std::vector<std::array<double, 2>> single_path;
        for (auto local_point : local_solu) {
            auto global_point = 
                    Local2Global(local_point, {x_0, y_0}, heading);
            single_path.push_back(global_point);
        }
        final_solution.push_back(single_path);
    }
    for (auto& dest : local_destinaton) {
        auto temp_dest = Local2Global(dest, {x_0, y_0}, heading);
        destination.push_back(temp_dest);
    }

    clock_t end_time = clock();

    double run_time = (double)(end_time - start_time) * 1000 / CLOCKS_PER_SEC;
    std::cout << "Successfully get the local path in " 
              << run_time << " ms" << std::endl;

    return final_solution;
}
}