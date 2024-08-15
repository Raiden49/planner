/*
 * @Author: Raiden49 
 * @Date: 2024-06-26 10:26:24 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-08-15 16:02:17
 */
#include "optim/b_spline_optim.hpp"

namespace optimization 
{
double BSpline::BasisFun(const int& i, const int& k, const double& t,
                         const std::vector<double>& knots) {
    if (k == 1) {
        return (t >= knots[i] && t < knots[i + 1]) ? 1.0 : 0.0;
    }

    double left_molecule = t - knots[i];
    double left_denominator = knots[i + k] - knots[i];
    double right_molecule = knots[i + k + 1] - t;
    double right_denominator = knots[i + k + 1] - knots[i + 1];

    double left_coeff, right_coeff;
    // 计算基函数时，如果分子为０，整个分数为０，如果分母为０，分母设置为１
    if (left_denominator == 0) {
        if (left_molecule == 0) {
            left_coeff = 0;
        }
        left_coeff = left_molecule / 1.;
    }
    else {
        left_coeff = left_molecule / left_denominator;
    }

    if (right_denominator == 0) {
        if (right_molecule == 0) {
            right_coeff = 0;
        }
        right_coeff = right_molecule / 1.;
    }
    else {
        right_coeff = right_molecule / right_denominator;
    }

    return left_coeff * BasisFun(i, k - 1, t, knots) + 
           right_coeff * BasisFun(i + 1, k - 1, t, knots);
}
std::vector<double> BSpline::GenerateKnots(const int& k, 
                                           const int& num_control_points) {
    std::vector<double> knots(num_control_points + k + 1);
    for (int i = 0; i < k; i++) {
        knots[i] = 0.0;
    }
    for (int i = k; i <= num_control_points; i++) {
        knots[i] = static_cast<double>(i - k + 1) / (num_control_points - k + 1);
    }
    for (int i = num_control_points + 1; i < num_control_points + k + 1; i++) {
        knots[i] = 1.;
    }

    return knots;
}
std::vector<Point3d> BSpline::Process() {
    std::vector<Point3d> solution;

    clock_t start_time = clock();

    int k = 3, num_control_point = path_ptr_->size();
    std::vector<double> knots = GenerateKnots(k, num_control_point);

    for (int j = 0; j < num_samples_; j++) {
        Point3d point({0.0, 0.0});
        double t = static_cast<double>(j) / num_samples_ * 
                (knots[knots.size() - 1] - knots[0]) + knots[0];
        for (int i = 0; i < num_control_point; i++) {
            double basis = BasisFun(i, k, t, knots);
            point.x += basis * path_ptr_->at(i).x;
            point.y += basis * path_ptr_->at(i).y;
        }
        solution.push_back(point);
    }

    clock_t end_time = clock();
    double run_time = (double)(end_time - start_time) * 1000 / CLOCKS_PER_SEC;
    // std::cout << "Get the solution in " << run_time << " ms" << std::endl;

    return solution;  
}
}