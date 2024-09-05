/*
 * @Author: Raiden49 
 * @Date: 2024-09-04 11:02:59 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-09-04 17:45:47
 */
#ifndef M_PRM_HPP_
#define M_PRM_HPP_

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <Eigen/Dense>

#include "m_util.hpp"

namespace m_prm {
class PRM {
    public:
        PRM(ros::NodeHandle& nh) : nh_(nh) {
            is_tree_built_ = false;
            is_map_recevied_ = false;
            map_sub_ = nh_.subscribe("/map", 10, &PRM::MapCallBack, this);
        }
        ~PRM() = default;
        
        void MapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg);
        std::vector<m_util::Point2t<int>> GenerateSamplePoints();
        bool RangeSearch(const std::shared_ptr<m_util::KDTreeNode<int>> node, 
                         const m_util::Point2t<int>& target,
                         std::vector<m_util::Point2t<int>>& result);
        bool PRMDisplay(visualization_msgs::MarkerArray& marker_array);
        bool IsMapReceived() {return is_map_recevied_;}
        bool IsTreeBuilt() {return is_tree_built_;}
    private:
        bool is_map_recevied_, is_tree_built_;
        int range_thread_ = 50;
        ros::NodeHandle nh_;
        Eigen::MatrixXi pgm_map_ = Eigen::Matrix<int, 1, 1>();
        std::shared_ptr<m_util::MapInfoTool> 
                map_tool_ = std::make_shared<m_util::MapInfoTool>();
        ros::Subscriber map_sub_;
};
}

#endif // M_PRM_HPP_