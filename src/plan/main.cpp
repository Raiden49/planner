/*
 * @Author: Raiden49 
 * @Date: 2024-06-26 10:26:16 
 * @Last Modified by:   Raiden49 
 * @Last Modified time: 2024-06-26 10:26:16 
 */
#include "plan/plan.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "plan");
    ros::NodeHandle nh;
    auto node = new plan::Plan(nh);
    node->Process();

    return 0;
}