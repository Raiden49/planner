/*
 * @Author: Raiden49 
 * @Date: 2024-06-26 10:25:37 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-09-04 17:09:06
 */
#ifndef M_UTIL_HPP_
#define M_UTIL_HPP_

#include <cmath>
#include <iostream>
#include <vector>
#include <queue>
#include <array>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <chrono>
#include <Eigen/Dense>

//　全是工具类函数
namespace m_util 
{
template<typename T>
struct Point2t {
    T x;
    T y;
    Point2t() : x(0), y(0) {};
    Point2t(T x, T y) : x(x), y(y) {};
};
template<typename T>
struct KDTreeNode {
    int axis;
    Point2t<T> point;
    std::shared_ptr<KDTreeNode> left;
    std::shared_ptr<KDTreeNode> right;
    KDTreeNode(const Point2t<T>& point, int axis) : 
            point(point), left(nullptr), right(nullptr), axis(axis) {};
};

template<typename T>
std::shared_ptr<KDTreeNode<T>> BuildKDTree(std::vector<Point2t<T>>& points, 
                                           int depth = 0) {
    if (points.empty()) {
        return nullptr;
    }                  
    int axis = depth % 2;
    std::sort(points.begin(), points.end(), [axis](const Point2t<T>& a, const Point2t<T>& b) {
        return (axis == 0) ? a.x < b.x : a.y < b.y;
    }); 
    
    size_t mid_index = points.size() / 2;
    Point2t<T> mid_point = points[mid_index];
    auto node = std::make_shared<KDTreeNode<T>>(mid_point, axis);

    std::vector<Point2t<T>> left_points(
            points.begin(), points.begin() + mid_index);
    std::vector<Point2t<T>> right_points(
            points.begin() + mid_index + 1, points.end());

    node->left = BuildKDTree(left_points, depth + 1);
    node->right = BuildKDTree(right_points, depth + 1);

    return node;
}

// 求欧式距离
template<typename T>
inline double EuclideanDis(const T& x_1, const T& y_1, const T& x_2, const T& y_2) {
    return sqrt(pow(x_1 - x_2, 2) + pow(y_1 - y_2, 2));
}
// 求曼哈顿距离
template<typename T>
inline double ManhattanDis(const T& x_1, const T& y_1, const T& x_2, const T& y_2) {
    return abs(x_1 - x_2) + abs(y_1 - y_2);
}
// 求对角线距离
template<typename T>
inline double DiagonalDis(const T& x_1, const T& y_1, const T& x_2, const T& y_2) {
    return sqrt(2) * 
           abs(x_1 - x_2) > abs(y_1 - y_2) ? abs(y_1 - y_2) : abs(x_1 - x_2) + 
           abs(x_1 - x_2) + abs(y_1 - y_2) - 2 * 
           abs(x_1 - x_2) > abs(y_1 - y_2) ? abs(y_1 - y_2) : abs(x_1 - x_2);
}
// 求距离当前位置最短的路径点编号
int GetMinDisIndex(int& start_num, const std::array<double, 2>& current_pose, 
                   const std::vector<std::array<double, 2>>& path_vector); 
//　创建可视化的marker
visualization_msgs::Marker CreateVisualMarker(
        const double& alpha, const std::array<double ,3>& color, 
        const std::array<double ,2>& scale, const std::string& frame_id, 
        const std::string& type, const int& id = 0);     

struct Point3d {
    double x;
    double y;
    double yaw;
    Point3d() : x(0), y(0), yaw(0) {};
    Point3d(double x, double y, double yaw) : x(x), y(y), yaw(yaw) {};
    Point3d(Point2t<double> point) : x(point.x), y(point.y), yaw(0) {};
};

class MapInfoTool {
    public:
        MapInfoTool() : map_ptr_(nullptr) {
            ros::param::get("origin_x", origin_x_);
            ros::param::get("origin_y", origin_y_);
            ros::param::get("resolution", resolution_);
        }
        ~MapInfoTool() = default;

        void set_map_ptr_(const Eigen::MatrixXi& map) {
            map_ptr_ = std::make_shared<const Eigen::MatrixXi>(map);
        }
        bool IsLineAvailable(int x_1, int y_1, int x_2, int y_2);
    public:
        inline m_util::Point2t<int> World2Map(const double& x, const double& y) {
            int map_x = (x - origin_x_) / resolution_;
            int map_y = (y - origin_y_) / resolution_;
            return m_util::Point2t<int>(map_x, map_y);
        }
        inline m_util::Point2t<double> Map2World(const int& x, const int& y) {
            double world_x = x * resolution_ + origin_x_;
            double world_y = y * resolution_ + origin_y_;
            return m_util::Point2t<double>(world_x, world_y);
        }
        inline bool InBoundary(const int& x, const int& y) {
            return x >= 0 && x < map_ptr_->rows() && y >= 0 && y <  map_ptr_->cols();
        }
        inline bool HasObstacle(const int& x, const int& y) {
            return (*map_ptr_)(x, y) != 0;
        }
    private:
        double resolution_, origin_x_, origin_y_;
        std::shared_ptr<const Eigen::MatrixXi> map_ptr_;
};   
}


#endif // M_UTIL_HPP_