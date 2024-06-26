/*
 * @Author: Raiden49 
 * @Date: 2024-06-26 10:26:26 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-06-26 11:44:37
 */
#include "optim/bezier_optim.hpp"

namespace optimization
{
int Bezier::factorial(const int& n) {
    if (n <= 1) {
        return 1;
    }
    return n * factorial(n - 1);
}

int Bezier::Binomial(const int& n, const int& i) {
    return factorial(n) / (factorial(i) * factorial(n - i));
}

std::vector<std::array<double, 2>> Bezier::GenerateControlPoints() {
    std::vector<std::array<double, 2>> control_points;
    control_points.push_back(path_ptr_->at(0));

    if (path_ptr_->size() < 2) {
        std::cout << "Error path size for bezier" << std::endl;
        return control_points;
    }

    // 这里就是很简单的数学计算，下面的方法是化简后得到的
    for (int i = 0; i < path_ptr_->size() - 1; i++) {
        auto p0 = path_ptr_->at(i);
        auto p1 = p0, p2 = p0;
        auto p3 = path_ptr_->at(i + 1);

        if (i == 0) {
            p1 = {(2 * p0[0] + p3[0]) / 3, (2 * p0[1] + p3[1]) / 3};
        } 
        else {
            auto last_p1 = control_points.at(control_points.size() - 2);
            p1 = {2 * p0[0] - last_p1[0], 2 * p0[1] - last_p1[1]};
        }

        p2 = {(p0[0] + 2 * p3[0]) / 3, (p0[1] + 2 * p3[1]) / 3};

        control_points.push_back(p1);
        control_points.push_back(p2);
        control_points.push_back(p3);

    }

    return control_points;
}

std::vector<std::array<double, 2>> Bezier::Process() {
    std::vector<std::array<double, 2>> solution;

    clock_t start_time = clock();

    auto control_points = GenerateControlPoints();

    // 3次贝塞尔，因此每次只处理４个控制点
    int index = 0, n = 3;
    while (1) {
        if (index + n > control_points.size() - 1) {
            break;
        }
        for (int m = 0; m <= num_samples_; m++) {
            double t = static_cast<double>(m) / num_samples_;
            std::array<double, 2> point{0.0, 0.0};
            for (int i = 0; i <= n; i++) {
                double factor = Binomial(n, i) * pow(t, i) * pow(1 - t, n - i);
                point[0] += control_points.at(index + i)[0] * factor;
                point[1] += control_points.at(index + i)[1] * factor;
            }
            solution.push_back(point);
        }
        index += n;
    }

    clock_t end_time = clock();
    double run_time = (double)(end_time - start_time) * 1000 / CLOCKS_PER_SEC;
    // std::cout << "Get the solution in " << run_time << " ms" << std::endl;

    return solution;
}
}