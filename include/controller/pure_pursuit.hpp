/*
 * @Author: Raiden49 
 * @Date: 2024-06-26 10:26:01 
 * @Last Modified by:   Raiden49 
 * @Last Modified time: 2024-06-26 10:26:01 
 */
#ifndef PURE_PURSUIT_HPP_
#define PURE_PURSUIT_HPP_

#include "controller/controller_interface.hpp"

namespace controller
{
class PurePursuit : public ControllerInterface {
    public:
        PurePursuit(const std::vector<std::array<double, 2>>& path,
                const std::array<double, 2>& current_pos,
                const double& current_radian) : 
                ControllerInterface(path, current_pos, current_radian) {
            this->current_radian_ = current_radian;
            this->current_pos_ = current_pos;
        }
        ~PurePursuit() {};

        bool ControlProcess(double& linear, double& angular_vel);

    public:
        std::array<double, 2> current_pos_;
        double current_radian_;
        int last_index_ = 0;
};
}

#endif // PURE_PURSUIT_HPP_