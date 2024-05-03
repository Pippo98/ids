#include <Eigen/Core>
#include <cstdlib>
#include <iostream>
#include <unsupported/Eigen/MatrixFunctions>

#include "Eigen/src/Core/Matrix.h"
#include "kalman_filter/ekf.hpp"
#include "kalman_filter/kf.hpp"
#include "kalman_filter/kf_base.hpp"
#include "kalman_filter/ukf.hpp"
#include "raylib.h"
#include "raymath.h"

int main(void) {
  srand(1);
  size_t N = 1000;
  double DT = 0.01;

  Eigen::VectorXd state(4);
  state.setZero();
  state(1) = state(3) = 1.0;

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

  auto stateUpdate = [](const Eigen::VectorXd &statePrev,
                        const Eigen::VectorXd &inputs, void *userData) {
    double DT = *(double *)userData;
    auto state = statePrev;
    state(0) = statePrev(0) + statePrev(1) * DT;
    state(1) = statePrev(1);
    state(2) = statePrev(2) + statePrev(3) * DT;
    state(3) = statePrev(3);
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
    jacobianH.setZero();
    jacobianH(0, 0) = 1;
    jacobianH(1, 2) = 1;
    return jacobianH;
  };

  KalmanFilter kf;
  kf.setState(state);
  kf.setStateCovariance(P);
  kf.setStateUpdateMatrix(A);
  kf.setMeasurementMatrix(H);
  kf.setProcessCovariance(Q);
  kf.setMeasurementCovariance(R);

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

  UnscentedKalmanFilter ukf;
  ukf.setState(state);
  ukf.setUserData(&DT);
  ukf.setStateCovariance(P);
  ukf.setProcessCovariance(Q);
  ukf.setMeasurementCovariance(R);
  ukf.setStateUpdateFunction(stateUpdate);
  ukf.setMeasurementFunction(measurementFunction);

  Vector2 start{0, 0};
  Vector2 end{100, 100};
  for (size_t i = 0; i <= N; i++) {
    kf.KalmanFilterBase::predict();
    ekf.KalmanFilterBase::predict();
    ukf.KalmanFilterBase::predict();

    if (i % 10 == 0) {
      float dist = Vector2Distance(start, end) * i / (float)N;
      auto pos = Vector2MoveTowards(start, end, dist);
      pos.x += rand() / (float)RAND_MAX * 0.5;
      pos.y += rand() / (float)RAND_MAX * 0.5;
      Eigen::Vector2d measurements{pos.x, pos.y};
      kf.update(measurements);
      ekf.update(measurements);
      ukf.update(measurements);
    }
  }
  printf("\n-- KF --\n");
  kf.print();
  printf("-- EKF --\n");
  ekf.print();
  printf("-- UKF --\n");
  ukf.print();

  return 0;
}
