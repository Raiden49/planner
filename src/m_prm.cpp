/*
 * @Author: Raiden49 
 * @Date: 2024-09-04 11:13:23 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-09-04 17:47:57
 */
#include "m_prm.hpp"

namespace m_prm
{
void PRM::MapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    is_map_recevied_ = true;
    int width = msg->info.width, height = msg->info.height;
    pgm_map_ = Eigen::MatrixXi::Zero(height, width);
    for (int row = 0; row < height; row++) {
        for (int col = 0; col < width; col++) {
            pgm_map_(row, col) = msg->data[col * width + row];
        }
    }
    map_tool_->set_map_ptr_(pgm_map_);
}
std::vector<m_util::Point2t<int>> PRM::GenerateSamplePoints() {
    std::vector<m_util::Point2t<int>> sample_points;
    Eigen::MatrixXi sampled_map = Eigen::MatrixXi::Zero(pgm_map_.rows(), pgm_map_.cols());
    
    int count = 1000;
    srand((unsigned) time(NULL));
    while (count--) {
        int x = rand() % pgm_map_.rows();
        int y = rand() % pgm_map_.cols();
        if (pgm_map_(x, y) != 0) {
            sampled_map(x, y) = 1;
        }
        if (sampled_map(x, y) == 1) {
            continue;
        }
        sampled_map(x, y) = 1;
        sample_points.push_back(m_util::Point2t<int>(x, y));
    }
    return sample_points;
}
bool PRM::RangeSearch(const std::shared_ptr<m_util::KDTreeNode<int>> node, 
                      const m_util::Point2t<int>& target,
                      std::vector<m_util::Point2t<int>>& result) {
    if (!node) {
        std::cout << "KD Tree is null!!!" << std::endl;
        return false;
    }

    double dis = m_util::EuclideanDis(node->point.x, node->point.y, target.x, target.y);
    if (dis < range_thread_) {
        result.push_back(node->point);
    }

    int diff = (node->axis == 0) ? target.x - node->point.x : target.y - node->point.y;
    if (diff < 0) {
        if (node->left) {
            RangeSearch(node->left, target, result);
        }
        if (diff * diff <= range_thread_ * range_thread_) {
            if (node->right) {
                RangeSearch(node->right, target, result);
            }
        }
    }
    else {
        if (node->right) {
            RangeSearch(node->right, target, result);
        }
        if (diff * diff <= range_thread_ * range_thread_) {
            if (node->left) {
                RangeSearch(node->left, target, result);
            }
        }
    }
    return true;
}
bool PRM::PRMDisplay(visualization_msgs::MarkerArray& marker_array) {
    auto points_marker = m_util::
            CreateVisualMarker(1.0, {1., 0., 0.}, {0.1, 0.1}, "map", "point", 0);
    auto lines_marker = m_util::
            CreateVisualMarker(1.0, {0., 1, 0.}, {0.02, 0.02}, "map", "line_list", 1);
    
    auto&& sampled_points = GenerateSamplePoints();
    auto root = m_util::BuildKDTree(sampled_points);
    is_tree_built_ = root ? true : false;

    for (auto& point : sampled_points) {
        auto world_point = map_tool_->Map2World(point.x, point.y);
        geometry_msgs::Point p1;
        p1.x = world_point.x; p1.y = world_point.y;
        points_marker.points.push_back(p1);
        
        std::vector<m_util::Point2t<int>> result;
        if (RangeSearch(root, point, result)) {
            for (auto& target : result) {
                auto world_target = map_tool_->Map2World(target.x, target.y);
                geometry_msgs::Point p2;
                p2.x = world_target.x; p2.y = world_target.y;
                if (map_tool_->IsLineAvailable(point.x, point.y, target.x, target.y)) {
                    lines_marker.points.push_back(p1);
                    lines_marker.points.push_back(p2);
                }
            }
        }
        else {
            return false;
        }
    }
    marker_array.markers.push_back(points_marker);
    marker_array.markers.push_back(lines_marker);
    return true;
}
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "prm");
    ros::NodeHandle nh;
    
    ros::Publisher prm_pub = 
            nh.advertise<visualization_msgs::MarkerArray>("prm_markers", 1); 
    auto prm_node = std::make_shared<m_prm::PRM>(nh);
    visualization_msgs::MarkerArray marker_array;
    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        if (!prm_node->IsMapReceived()) {
            continue;
        }
        if (!prm_node->IsTreeBuilt()) {
            if (!prm_node->PRMDisplay(marker_array)) {
                return 0;
            }
        }
        prm_pub.publish(marker_array);
        
        rate.sleep(); 
    }
    return 0;
}
