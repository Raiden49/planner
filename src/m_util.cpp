/*
 * @Author: Raiden49 
 * @Date: 2024-06-26 10:26:09 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-06-26 10:27:41
 */
#include "m_util.hpp"

namespace m_util
{
inline double EuclideanDis(const double& x_1, const double& y_1, 
                    const double& x_2, const double& y_2) {
    return sqrt(pow(x_1 - x_2, 2) + pow(y_1 - y_2, 2));
}

inline double ManhattanDis(const double& x_1, const double& y_1, 
                    const double& x_2, const double& y_2) {
    return abs(x_1 - x_2) + abs(y_1 - y_2);
}

inline double DiagonalDis(const double& x_1, const double& y_1, 
                   const double& x_2, const double& y_2) {
    return sqrt(2) * 
           abs(x_1 - x_2) > abs(y_1 - y_2) ? abs(y_1 - y_2) : abs(x_1 - x_2) + 
           abs(x_1 - x_2) + abs(y_1 - y_2) - 2 * 
           abs(x_1 - x_2) > abs(y_1 - y_2) ? abs(y_1 - y_2) : abs(x_1 - x_2);
}

int GetMinDisIndex(int& start_num, const std::array<double, 2>& current_pose, 
                   const std::vector<std::array<double, 2>>& path_vector) {

    double min_dis = INFINITY;
    int index = 0;
    for (int i = start_num; i < path_vector.size(); i++) {
        double distance = EuclideanDis(path_vector[i][0], path_vector[i][1], 
                                       current_pose[0], current_pose[1]);
        if (distance < min_dis) {
            min_dis = distance;
            index = i;
        }
    }

    return index;
}

visualization_msgs::Marker CreateVisualMarker(
        const double& alpha, const std::array<double ,3>& color, 
        const std::array<double ,2>& scale, const std::string& frame_id, 
        const std::string& type, const int& id) {
    visualization_msgs::Marker new_marker;
    if (type == "line") {
        new_marker.type = visualization_msgs::Marker::LINE_STRIP;
    }
    if (type == "point") {
        new_marker.scale.y = scale[1];
        new_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    }
    new_marker.header.frame_id = frame_id;
    new_marker.header.stamp = ros::Time::now();
    new_marker.ns = frame_id;
    new_marker.id = id;
    new_marker.action = visualization_msgs::Marker::ADD;
    new_marker.lifetime = ros::Duration();
    new_marker.color.r = color[0];
    new_marker.color.g = color[1];
    new_marker.color.b = color[2];
    new_marker.color.a = alpha;
    new_marker.scale.x = scale[0];
    new_marker.pose.orientation.w = 1.0;

    return new_marker;
}
}
