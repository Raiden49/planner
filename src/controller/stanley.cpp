#include "controller/stanley.hpp"

namespace controller
{
bool Stanley::ControlProcess(double& linear, double& angular_vel) {

    if (path_ptr_->size() == 0) {
        return false;
    }

    double factor = 0.1;
    double speed = 0.5;
    
    std::cout << "last_index_: " << last_index_ << std::endl;

    int current_index = 
            m_util::GetMinDisIndex(last_index_, current_pos_, *path_ptr_);
    last_index_ = current_index;

    std::cout << "current_index: " << current_index << std::endl;

    std::array<double, 2> current_path_point = path_ptr_->at(current_index);

    double e_y = m_util::EuclideanDis(
            current_pos_[0], current_pos_[1], 
            current_path_point[0], current_path_point[1]);

    e_y = ((current_pos_[1] - current_path_point[1]) * 
            cos(current_radian_) - 
            (current_pos_[0] - current_path_point[0]) * 
            sin(current_radian_)) <= 0 ? e_y : -e_y; 
    
    double path_radian = current_index > 0 ? atan2(
            path_ptr_->at(current_index)[1] - path_ptr_->at(current_index - 1)[1], 
            path_ptr_->at(current_index)[0] - path_ptr_->at(current_index - 1)[0]) : 
            atan2(path_ptr_->at(current_index)[1], 
                  path_ptr_->at(current_index)[0]);
            
    double theta_e = path_radian - current_radian_;

    double delta_e = atan2(factor * e_y, speed);
    double delta = delta_e + theta_e * 0.3;

    // delta = theta_e;
    std::cout << "current_radian:" << current_radian_ * 180 / M_PI << std::endl;
    std::cout << "target_radian:" << path_radian * 180 / M_PI << std::endl;
        
    if (delta > M_PI) {
        delta -= 2 * M_PI;
    }
    else if (delta < -M_PI) {
        delta += 2 * M_PI;
    }

    linear = speed;
    angular_vel = delta;

    if (current_index == path_ptr_->size() - 1) {
        linear = 0;
        angular_vel = 0;
    }

    return true;
}
}
