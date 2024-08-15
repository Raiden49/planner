/*
 * @Author: Raiden49 
 * @Date: 2024-06-26 10:24:48 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-08-15 16:02:19
 */
#ifndef QP_OPTIM_HPP_
#define QP_OPTIM_HPP_

#include "optim/optim_interface.hpp"

namespace optimization
{
/**
 * @brief 二次规划实现路径优化，基于Osqp-Eigen，三个约束项目
 */
class QP : public OptimInterface {
    public:
        QP(const double& lower_bound, const double& upper_bound,
           const double& weight_smooth, const double& weight_length, 
           const double& weight_ref, 
           const std::vector<Point3d>& path) : 
           OptimInterface(path), 
           lower_bound_(lower_bound), upper_bound_(upper_bound),
           weight_smooth_(weight_smooth), weight_length_(weight_length),
           weight_ref_(weight_ref) {};
        ~QP() = default;
        
        bool InitSolver(Eigen::SparseMatrix<double>& hessian, 
                        Eigen::SparseMatrix<double>& linear_matrix,
                        Eigen::VectorXd& gradient,
                        Eigen::VectorXd& lower_bound,
                        Eigen::VectorXd& upper_bound); 
        std::vector<Point3d> Process() override;
    
    private:
        double lower_bound_, upper_bound_;
        double weight_smooth_, weight_length_, weight_ref_;
};
}

#endif // QP_OPTIM_HPP_