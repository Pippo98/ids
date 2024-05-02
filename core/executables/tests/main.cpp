#include <Eigen/Core>
#include <cstdlib>
#include <iostream>
#include <unsupported/Eigen/MatrixFunctions>

#include "Eigen/src/Core/Matrix.h"
#include "kalman_filter/ekf.hpp"
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
  Eigen::MatrixXd R(2, 2);
  R.setIdentity();
  R = R * 0.1;

  kf.setState(state);
  kf.setStateCovariance(P);
  kf.setStateUpdateMatrix(A);
  kf.setMeasurementMatrix(H);
  kf.setProcessCovariance(Q);
  kf.setMeasurementCovariance(R);

  Vector2 start;
  Vector2 end{100, 100};
  double speed = Vector2Distance(start, end) / (N * DT);

  auto stateUpdate = [](const Eigen::VectorXd &statePrev,
                        const Eigen::VectorXd &inputs, void *userData) {
    double DT = *(double *)userData;
    auto state = statePrev;
    state(0) = statePrev(0) + statePrev(1) * DT;
    state(2) = statePrev(2) + statePrev(3) * DT;
    return state;
  };
  auto measurementFunction = [](const Eigen::VectorXd &state, void *userData) {
    Eigen::VectorXd measures(2);
    measures(0) = state(0);
    measures(1) = state(2);
    return measures;
  };

  auto stateJacobianFunction = [](const Eigen::VectorXd &state,
                                  const Eigen::VectorXd &inputs,
                                  void *userData) {
    double DT = *(double *)userData;
    Eigen::MatrixXd jacobianF(4, 4);
    jacobianF.setIdentity();
    jacobianF(0, 1) = DT;
    jacobianF(2, 3) = DT;
    return jacobianF;
  };
  auto measureJacobianFunction = [](const Eigen::VectorXd &state,
                                    void *userData) {
    Eigen::MatrixXd jacobianH(2, 4);
    jacobianH(0, 0) = 1;
    jacobianH(1, 2) = 1;
    return jacobianH;
  };

  ExtendedKalmanFilter ekf;
  ekf.setState(state);
  ekf.setUserData(&DT);
  ekf.setStateUpdateFunction(stateUpdate);
  ekf.setMeasurementFunction(measurementFunction);
  ekf.setStateJacobian(stateJacobianFunction);
  ekf.setMeasurementJacobian(measureJacobianFunction);
  ekf.setStateCovariance(P);
  ekf.setProcessCovariance(Q);
  ekf.setMeasurementCovariance(R);

  for (size_t i = 0; i <= N; i++) {
    kf.predict();
    ekf.predict();

    if (i % 10 == 0) {
      float dist = Vector2Distance(start, end) * i / (float)N;
      auto pos = Vector2MoveTowards(start, end, dist);
      pos.x += rand() / (float)RAND_MAX * 0.5;
      pos.y += rand() / (float)RAND_MAX * 0.5;
      Eigen::Vector2d measurements{pos.x, pos.y};
      kf.update(measurements);
      ekf.update(measurements);
    }
    printf("\n-- KF --\n");
    kf.print();
    printf("-- EKF --\n");
    ekf.print();
  }
  // kf.print();
  // ekf.print();

  return 0;
}
