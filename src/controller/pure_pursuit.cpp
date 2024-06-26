#include "controller/pure_pursuit.hpp"

namespace controller
{
bool PurePursuit::ControlProcess(double& linear, double& angular_vel) {

    if (path_ptr_->size() == 0) {
        return false;
    }

    double k =  3, speed = 0.5, lf = 1;
    double ld = k * speed + lf;
    
    auto ahead_pos = current_pos_;
    ahead_pos[0] = current_pos_[0] + ld * sin(current_radian_);
    ahead_pos[1] = current_pos_[1] + ld * cos(current_radian_);

    int ahead_index = m_util::GetMinDisIndex(last_index_, ahead_pos, *path_ptr_);
    last_index_ = ahead_index;

    std::cout << "ahead index: " << ahead_index << std::endl;

    double ld_new;
    while (1) {
        double dis = m_util::EuclideanDis(current_pos_[0], current_pos_[1], 
                path_ptr_->at(ahead_index)[0], path_ptr_->at(ahead_index)[1]);  
        if (dis >= ld) {
            ld_new = dis;
            break;
        }         
        ahead_index += 1;
        if (ahead_index >= path_ptr_->size() - 1) {
            ahead_index = path_ptr_->size() - 1;
            break;
        }
    }

    auto ahead_path_point = path_ptr_->at(ahead_index);

        // delta = atan2(2Lsin(alpha) / ld)(L = 0.5)
        double alpha = std::atan2(ahead_path_point[1] - current_pos_[1], 
                ahead_path_point[0] - current_pos_[0]) - current_radian_;

        double L = 2;
        double delta = std::atan2(2 * L * sin(alpha), ld_new);        

        linear = speed;
        angular_vel = delta;

    return true;
}
}