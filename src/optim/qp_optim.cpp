/*
 * @Author: Raiden49 
 * @Date: 2024-06-26 10:26:28 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-08-15 16:02:14
 */
#include "optim/qp_optim.hpp"

namespace optimization
{
bool QP::InitSolver(Eigen::SparseMatrix<double>& hessian, 
                    Eigen::SparseMatrix<double>& linear_matrix,
                    Eigen::VectorXd& gradient,
                    Eigen::VectorXd& lower_bound,
                    Eigen::VectorXd& upper_bound) {
    int n = path_ptr_->size();
    std::cout << "the path node size:" << n << std::endl;

    if (n <= 0) {
        std::cout << "Error path size!!!" << std::endl;
        return false;
    }

    Eigen::SparseMatrix<double> A1(2*n, 2*n-4);
    for (int row = 0; row < 2 * n; row++) {
        for (int col = 0; col < 2 * n - 4; col++) {
            if (row == col) {
                A1.insert(row, col) = 1;
            }
            else if (row - col == 2) {
                A1.insert(row, col) = -2;
            }
            else if (row - col == 4) {
                A1.insert(row, col) = 1;
            }
        }
    }
    Eigen::SparseMatrix<double> A2(2*n, 2*n-2);
    for (int row = 0; row < 2 * n; row++) {
        for (int col = 0; col < 2 * n - 2; col++) {
            if (row == col) {
                A2.insert(row, col) = 1;
            }
            else if (row - col == 2) {
                A2.insert(row, col) = -1;
            }
        }
    }
    Eigen::SparseMatrix<double> A3(2*n, 2*n);
    for (int row = 0; row < 2 * n; row++) {
        for (int col = 0; col < 2 * n; col++) {
            if (row == col) {
                A3.insert(row, col) = 1;
            }
        }
    }

    // 起点与终点不进行优化处理，即把他们对应的上下界设置为本身就可以
    for (int num = 0; num < 2 * n; num += 2) {
        gradient(num, 0) = -2 * path_ptr_->at(num / 2).x;
        gradient(num + 1, 0) = -2 * path_ptr_->at(num / 2).y;
        lower_bound(num, 0) = 
                path_ptr_->at(num / 2).x + (num == 0 ? 0 : lower_bound_);
        lower_bound(num + 1, 0) = 
                path_ptr_->at(num / 2).y + (num == 0 ? 0 : lower_bound_);
        upper_bound(num, 0) = 
                path_ptr_->at(num / 2).x + (num == 0 ? 0 : upper_bound_);
        upper_bound(num + 1, 0) = 
                path_ptr_->at(num / 2).y + (num == 0 ? 0 : upper_bound_);
    }

    hessian = 2. * (weight_smooth_ * A1 * A1.transpose() + 
            weight_length_* A2 * A2.transpose() + weight_ref_ * A3 * A3);
    linear_matrix = A3;
    gradient = weight_ref_ * gradient;

    return true;
}

std::vector<Point3d> QP::Process() {
    
    OsqpEigen::Solver solver;
    std::vector<Point3d> opti_path;

    int n = path_ptr_->size();
    Eigen::VectorXd gradient(2*n), lower_bound(2*n), upper_bound(2*n);
    Eigen::SparseMatrix<double> hessian, linear_matrix;

    if (!InitSolver(hessian, linear_matrix, gradient, lower_bound, upper_bound)) {
        std::cout << "Error initizing QP solver!!!" << std::endl;
    }

    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
    try {
        solver.data()->setNumberOfVariables(2*n);
        solver.data()->setNumberOfConstraints(2*n);
        solver.data()->setHessianMatrix(hessian);
        solver.data()->setLinearConstraintsMatrix(linear_matrix);
        solver.data()->setGradient(gradient);
        solver.data()->setLowerBound(lower_bound);
        solver.data()->setUpperBound(upper_bound);
    } catch (const std::exception& e) {
        std::cerr << "Error setting solver data!!!" << e.what() << std::endl;
    }

    clock_t start_time = clock();
    try {
        solver.initSolver();
        solver.solve();
    } catch (const std::exception& e) {
        std::cerr << "Error getting problem solved" << e.what() << std::endl;
    }
    clock_t end_time = clock();
    double run_time = (double)(end_time - start_time) * 1000 / CLOCKS_PER_SEC;
    // std::cout << "Get the solution in " << run_time << " ms" << std::endl;

    Eigen::VectorXd solution = solver.getSolution();
    for (int i = 0; i < solution.size(); i += 2) {
        opti_path.push_back(Point3d({solution(i, 0), solution(i + 1, 0)}));
    }
    return opti_path;
}
}