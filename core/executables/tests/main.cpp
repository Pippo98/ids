#include <Eigen/Core>
#include <cstdlib>
#include <iostream>
#include <unsupported/Eigen/MatrixFunctions>

#include "Eigen/src/Core/Matrix.h"
#include "kalman_filter/kf.hpp"
#include "raylib.h"
#include "raymath.h"

int main(void) {
  size_t N = 1000;
  double DT = 0.01;

  KalmanFilter kf;
  Eigen::VectorXd state(4);
  Eigen::MatrixXd A(4, 4);
  A.setIdentity();
  A(0, 1) = DT;
  A(2, 3) = DT;

  Eigen::MatrixXd P(4, 4);
  P.setIdentity();
  P *= 0.1;
  auto Q = P;
  Eigen::MatrixXd H(2, 4);
  H(0, 0) = 1;
  H(1, 2) = 1;

  kf.setState(state);
  kf.setStateCovariance(P);
  kf.setStateUpdateMatrix(A);
  kf.setMeasurementMatrix(H);
  kf.setProcessCovariance(Q);

  Vector2 start;
  Vector2 end{100, 100};
  double speed = Vector2Distance(start, end) / (N * DT);

  for (size_t i = 0; i <= N; i++) {
    kf.predict();

    if (i % 10 == 0) {
      float dist = Vector2Distance(start, end) * i / (float)N;
      auto pos = Vector2MoveTowards(start, end, dist);
      pos.x += rand() / (float)RAND_MAX * 0.5;
      pos.y += rand() / (float)RAND_MAX * 0.5;
      Eigen::Vector2d measurements{pos.x, pos.y};
      kf.update(measurements);
    }

    kf.print();
  }

  return 0;
}
