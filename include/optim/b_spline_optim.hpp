/*
 * @Author: Raiden49 
 * @Date: 2024-06-26 10:24:56 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-08-15 16:02:22
 */
#ifndef B_SPLINE_OPTIM_HPP_
#define B_SPLINE_OPTIM_HPP_

#include "optim/optim_interface.hpp"

namespace optimization
{
/**
 * @brief Ｂ样条曲线做路径优化，采用三次Ｂ样条曲线，用所有的路径点做控制点
 */
class BSpline : public OptimInterface {
    public:
        BSpline(const std::vector<Point3d>& path, const int& num_samples) : 
                OptimInterface(path), num_samples_(num_samples){};
        ~BSpline() = default;
    
    /**
     * @brief Ｂ样条曲线的基函数计算
     */
    double BasisFun(const int& i, const int& k, const double& t,
                    const std::vector<double>& konts);
    /**
     * @brief 生成节点，同时保证优化后路径经过原始路径的起点和终点
     * 
     * @return std::vector<double> 节点向量
     */
    std::vector<double> GenerateKnots(const int& k, 
                                      const int& num_control_points);
    
    std::vector<Point3d> Process() override;

    private:
        // 插值点数目，这里是所有路径点中的插值点数目
        int num_samples_;

};
}

#endif // B_SPLINE_OPTIM_HPP_