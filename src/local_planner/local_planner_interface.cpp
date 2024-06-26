/*
 * @Author: Raiden49 
 * @Date: 2024-06-26 10:26:31 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-06-26 11:45:00
 */
#include "local_planner/local_planner_interface.hpp"

namespace local_planner
{
std::array<double, 2> LocalPlannerInterface::Local2Global(
        const std::array<double, 2>& local_point, 
        const std::array<double, 2>& shift_point,
        const double& heading) {
    double theta = M_PI / 2 - heading;
    double global_x = shift_point[0]  
            + local_point[0] * cos(theta) + local_point[1] * sin(theta);
    double global_y = shift_point[1] 
            - local_point[0] * sin(theta) + local_point[1] * cos(theta);

    return {global_x, global_y};
}

std::array<double, 2> LocalPlannerInterface::Global2Local(
        const std::array<double, 2>& global_point, 
        const std::array<double, 2>& shift_point,
        const double& heading) {
    double theta = M_PI / 2 - heading;
    double local_x = (global_point[0] - shift_point[0]) * cos(theta) - 
            (global_point[1] - shift_point[1]) * sin(theta);
    double local_y = (global_point[0] - shift_point[0]) * sin(theta) + 
            (global_point[1] - shift_point[1]) * cos(theta);

    return {local_x, local_y};   
}

std::vector<std::array<double, 2>> LocalPlannerInterface::GetBestPath(
        int current_index,
        const std::vector<std::vector<std::array<double, 2>>>& paths) {

    std::shared_ptr<std::vector<std::array<double, 2>>> best_path_ptr;
    std::vector<double> score;
    double best_score = 0 - INFINITY;
    int best_num = -1;
    clock_t start_time = clock();
    for (auto &path : paths) {
        double obs_score = 0, ref_score = 0, path_score = 0;
        for (auto &point : path) {
            int temp_index = current_index;
            // obs score
            // 没有计算距离最近障碍物的距离，太耗时了，
            // 只计算一段范围内的障碍物信息，以像素坐标系计算
            auto map_point = World2Map(point[0], point[1]);
            for (int i = -4; i < 5; i++) {
                for (int j = -4; j < 5; j++) {
                    if ((*map_ptr_)(map_point[0] + i, map_point[1] + j) != 0) {
                        obs_score += 0 - (i * i + j * j) * pow(resolution_, 2);
                    }
                }
            }
        
            // ref score
            int count = 0;
            while (1) {
                count += 1;
                auto current_dis = 
                        m_util::EuclideanDis(point[0], point[1], 
                        path_ptr_->at(temp_index)[0], 
                        path_ptr_->at(temp_index)[1]);
                auto next_dis = temp_index + count < path_ptr_->size() ? 
                        m_util::EuclideanDis(point[0], point[1],
                        path_ptr_->at(temp_index + count)[0],
                        path_ptr_->at(temp_index + count)[1]) : 
                        current_dis;
                if (current_dis > next_dis) {
                    temp_index = temp_index + count;
                    count = 0;
                }
                if (count >= 10) {
                    ref_score += 0 - current_dis;
                    break;
                }
            }
        }
        // total score
        path_score = ref_score + obs_score;
        if (path_score > best_score) {
            best_score = path_score;
            best_path_ptr = std::make_shared<
                    std::vector<std::array<double, 2>>>(path);
        }
        score.push_back(path_score);
    }

    clock_t end_time = clock();
    double run_time = (double)(end_time - start_time) * 1000 / CLOCKS_PER_SEC;
    std::cout << "Successfully find the local best path in " 
              << run_time << " ms" << std::endl;

    return *best_path_ptr;
} 
}