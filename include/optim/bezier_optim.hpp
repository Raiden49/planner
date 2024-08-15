/*
 * @Author: Raiden49 
 * @Date: 2024-06-26 10:24:53 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-08-15 16:02:24
 */
#ifndef BEZIER_OPTIM_HPP_
#define BEZIER_OPTIM_HPP_

#include "optim/optim_interface.hpp"

namespace optimization
{
/**
 * @brief 贝塞尔曲线做路径优化，采用分段贝塞尔曲线，每一段都是三次贝塞尔
 */
class Bezier : public OptimInterface {
    public:
        Bezier(const std::vector<Point3d>& path, const int& num_samples) : 
               OptimInterface(path), num_samples_(num_samples) {};
        ~Bezier() = default;

        /**
         * @brief 计算ｎ的阶乘
         */
        int factorial(const int& n);
        /**
         * @brief 用于计算二项式
         */
        int Binomial(const int& n, const int& i);
        /**
         * @brief 生成控制点，不能直接用路径点作为控制点，否则质量很差，这里每两个路径点插入了两个控制点，同时保持前后连续
         * 
         * @return std::vector<std::array<double, 2>> 控制点向量
         */
        std::vector<Point3d> GenerateControlPoints();
        std::vector<Point3d> Process() override;

    private:
        //　差值点数目，这里是每两个路径点中的插值点数目
        int num_samples_;
};
}

#endif // BEZIER_OPTIM_HPP_