/*
 * @Author: Raiden49 
 * @Date: 2024-08-16 15:57:32 
 * @Last Modified by:   Raiden49 
 * @Last Modified time: 2024-08-16 15:57:32 
 */
#ifndef HYBRID_ASTAR_TYPE_HPP_
#define HYBRID_ASTAR_TYPE_HPP_

#include "global_planner/global_planner_interface.hpp"

namespace global_planner
{
struct HybridNode {
    enum NODE_STATUS {
        NOT_VISITED = 0, IN_OPENSET = 1, IN_CLOSESET = 2
    };
    enum DIRECTION {
        FORWARD = 0, BACKWARD = 1, NO = 3
    };

    int map_x_, map_y_;
    int steering_grade_;
    double g_cost_, h_cost_;
    Point3d world_point_;
    NODE_STATUS status_;
    DIRECTION direction_;
    std::shared_ptr<HybridNode> parent_ = nullptr;
    std::vector<Point3d> intermediate_point_;

    HybridNode() = delete;
    HybridNode(const int& map_x, const int& map_y, const Point3d& world_point) : 
               map_x_(map_x), map_y_(map_y), status_(NOT_VISITED), 
               parent_(nullptr), world_point_(world_point) {};
    double get_fcost() {return g_cost_ + 0.5 * h_cost_;}

    void Reset() {
        status_ = NOT_VISITED;
        parent_ = nullptr;
    }
};
} // namespace global_planner


#endif // HYBRID_ASTAR_TYPE_HPP_