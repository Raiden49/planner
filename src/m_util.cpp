/*
 * @Author: Raiden49 
 * @Date: 2024-06-26 10:26:09 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-09-04 16:15:03
 */
#include "m_util.hpp"

namespace m_util
{
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
    if (type == "line_list") {
        new_marker.type = visualization_msgs::Marker::LINE_LIST;
    }
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

bool MapInfoTool::IsLineAvailable(int x_1, int y_1, int x_2, int y_2) {
    // Bresenham 
    bool is_steep = abs(y_2 - y_1) > abs(x_2 - x_1);
    if (is_steep) {
        std::swap(x_1, y_1);
        std::swap(x_2, y_2);
    }
    if (x_1 > x_2) {
        std::swap(x_1, x_2);
        std::swap(y_1, y_2);
    }

    int delta_x = x_2 - x_1;
    int delta_y = abs(y_2 - y_1);
    double delta_error = (double)delta_y / delta_x;
    double error = 0;
    int y_step;
    auto y_k = y_1;
    if (y_1 < y_2) {
        y_step = 1;
    }
    else {
        y_step = -1;
    }
    auto num = (int) (x_2 - x_1);
    for (int i = 0; i < num; i++) {
        if (is_steep) {
            if (!InBoundary(y_k, x_1 + i) || HasObstacle(y_k, x_1 + i)) {
                return false;
            }
        } else {
            if (!InBoundary(x_1 + i, y_k) || HasObstacle(x_1 + i, y_k)) {
                return false;
            }
        }
        error += delta_error;
        if (error >= 0.5) {
            y_k += y_step;
            error = error - 1.0;
        }
    }
    return true;
}
}