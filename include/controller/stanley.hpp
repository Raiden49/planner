/*
 * @Author: Raiden49 
 * @Date: 2024-06-26 10:26:03 
 * @Last Modified by:   Raiden49 
 * @Last Modified time: 2024-06-26 10:26:03 
 */
#ifndef STANLEY_CORE_HPP_
#define STANLEY_CORE_HPP_

#include <sstream>
#include <cmath>
#include <std_srvs/Empty.h>

#include "controller/controller_interface.hpp"

namespace controller
{
class Stanley : public ControllerInterface {
    public:
        Stanley(const std::vector<std::array<double, 2>>& path,
                const std::array<double, 2>& current_pos,
                const double& current_radian) : 
                ControllerInterface(path, current_pos, current_radian) {
            this->current_radian_ = current_radian;
            this->current_pos_ = current_pos;
        }
        ~Stanley() {};

        bool ControlProcess(double& linear, double& angular_vel);

    public:
        std::array<double, 2> current_pos_;
        double current_radian_;
        int last_index_ = 0;
};
}

#endif //STANLEY_CORE_HPP_