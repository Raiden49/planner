/*
 * @Author: Raiden49 
 * @Date: 2024-06-26 10:25:58 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-06-26 11:16:39
 */
#ifndef CONTROLLER_INTERFACE_HPP_
#define CONTROLLER_INTERFACE_HPP_

#include <cmath>
#include <vector>
#include <array>
#include <iostream>
#include <memory>

#include "m_util.hpp"

namespace controller
{
/**
 * @brief 控制器接口，但是没用到控制器，有些不好调
 */
class ControllerInterface {
    public:
        ControllerInterface(const std::vector<std::array<double, 2>>& path,
                            const std::array<double, 2>& current_pos,
                            const double& current_radian) {
            path_ptr_ = 
                    std::make_shared<std::vector<std::array<double, 2>>>(path);
        };
        virtual ~ControllerInterface() {};
        virtual  bool ControlProcess(double& linear, double& angular_vel) = 0;
    public:
        std::shared_ptr<std::vector<std::array<double, 2>>> path_ptr_;
};
} // namespace controller


#endif // CONTROLLER_INTERFACE_HPP_