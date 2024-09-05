/*
 * @Author: Raiden49 
 * @Date: 2024-06-26 10:26:31 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-08-15 15:36:09
 */
#include "local_planner/local_planner_interface.hpp"

namespace local_planner
{
Point3d LocalPlannerInterface::Local2Global(const Point3d& local_point, 
                                            const Point3d& shift_point,
                                            const double& heading) {
    double theta = M_PI / 2 - heading;
    double global_x = shift_point.x  
            + local_point.x * cos(theta) + local_point.y * sin(theta);
    double global_y = shift_point.y 
            - local_point.x * sin(theta) + local_point.y * cos(theta);

    return Point3d(global_x, global_y, 0);
}

Point3d LocalPlannerInterface::Global2Local(const Point3d& global_point, 
                                            const Point3d& shift_point,
                                            const double& heading) {
    double theta = M_PI / 2 - heading;
    double local_x = (global_point.x - shift_point.x) * cos(theta) - 
            (global_point.y - shift_point.y) * sin(theta);
    double local_y = (global_point.x - shift_point.x) * sin(theta) + 
            (global_point.y - shift_point.y) * cos(theta);

    return Point3d(local_x, local_y, 0);   
}

std::vector<Point3d> LocalPlannerInterface::GetBestPath(
        int current_index,  const std::vector<std::vector<Point3d>>& paths) {

    std::shared_ptr<std::vector<Point3d>> best_path_ptr;
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
            auto map_point = map_tool_->World2Map(point.x, point.y);
            for (int i = -4; i < 5; i++) {
                for (int j = -4; j < 5; j++) {
                    if ((*map_ptr_)(map_point.x + i, map_point.y + j) != 0) {
                        obs_score += 0 - (i * i + j * j) * pow(resolution_, 2);
                    }
                }
            }
        
            // ref score
            int count = 0;
            while (1) {
                count += 1;
                auto current_dis = 
                        m_util::EuclideanDis(point.x, point.y, 
                        path_ptr_->at(temp_index).x, 
                        path_ptr_->at(temp_index).y);
                auto next_dis = temp_index + count < path_ptr_->size() ? 
                        m_util::EuclideanDis(point.x, point.y,
                        path_ptr_->at(temp_index + count).x,
                        path_ptr_->at(temp_index + count).y) : 
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
        path_score = 3 * ref_score + obs_score;
        if (path_score > best_score) {
            best_score = path_score;
            best_path_ptr = std::make_shared<std::vector<Point3d>>(path);
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