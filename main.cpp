#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>

int main() {
    std::cout << "Hello, World!" << std::endl;
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3, 3);
    std::cout << A << std::endl;
    return 0;
}
