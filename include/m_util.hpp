/*
 * @Author: Raiden49 
 * @Date: 2024-06-26 10:25:37 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-08-15 16:02:10
 */
#ifndef M_UTIL_HPP_
#define M_UTIL_HPP_

#include <cmath>
#include <iostream>
#include <vector>
#include <array>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <chrono>

//　全是工具类函数
namespace m_util 
{

struct Point3d {
    double x;
    double y;
    double yaw;
    Point3d() : x(0), y(0), yaw(0) {};
    Point3d(double x, double y, double yaw) : x(x), y(y), yaw(yaw) {};
    Point3d(std::array<double, 2> world_point) : x(world_point[0]), y(world_point[1]), yaw(0) {};
};

// 求欧式距离
double EuclideanDis(const double& x_1, const double& y_1, 
                    const double& x_2, const double& y_2);
// 求曼哈顿距离
double ManhattanDis(const double& x_1, const double& y_1, 
                    const double& x_2, const double& y_2);
// 求对角线距离
double DiagonalDis(const double& x_1, const double& y_1, 
                   const double& x_2, const double& y_2);
// 求距离当前位置最短的路径点编号
int GetMinDisIndex(int& start_num, const std::array<double, 2>& current_pose, 
                   const std::vector<std::array<double, 2>>& path_vector); 
//　创建可视化的marker
visualization_msgs::Marker CreateVisualMarker(
        const double& alpha, const std::array<double ,3>& color, 
        const std::array<double ,2>& scale, const std::string& frame_id, 
        const std::string& type, const int& id = 0);        
}

#endif // M_UTIL_HPP_