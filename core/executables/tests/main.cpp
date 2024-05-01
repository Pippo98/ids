#include <Eigen/Core>
#include <iostream>
#include <unsupported/Eigen/MatrixFunctions>

#include "kalman_filter/kf.hpp"

int main(void) {
  KalmanFilter kf;
  kf.print();
  return 0;
}
