#include <Eigen/LU>
#include <iostream>
#include <ostream>

#include "Eigen/src/Core/Matrix.h"
#include "ekf.hpp"
#include "kf.hpp"
#include "kf_base.hpp"

void KalmanFilterBase::setUserData(void *data) { userData = data; }

void KalmanFilterBase::setState(Eigen::VectorXd state) { X = state; }
void KalmanFilterBase::setStateCovariance(Eigen::MatrixXd stateCovariance) {
  P = stateCovariance;
}
void KalmanFilterBase::setProcessCovariance(Eigen::MatrixXd processCovariance) {
  Q = processCovariance;
}
void KalmanFilterBase::setMeasurementCovariance(
    Eigen::MatrixXd measurementCovariance) {
  R = measurementCovariance;
}

Eigen::VectorXd KalmanFilterBase::getState() { return X; }
Eigen::MatrixXd KalmanFilterBase::getCovariance() { return P; }

void KalmanFilterBase::print() { printToStream(std::cout); }
void KalmanFilterBase::printToStream(std::ostream &stream) {
  stream << "-- STATE --\n";
  stream << X << "\n";
  stream << "-- COVARIANCE --\n";
  stream << P << std::endl;
}

// Linear Kalman Filter
void KalmanFilter::setStateUpdateMatrix(const Eigen::MatrixXd &A_) {
  A = A_;
  B = Eigen::MatrixXd();
}

void KalmanFilter::setStateUpdateMatrices(const Eigen::MatrixXd &A_,
                                          const Eigen::MatrixXd &B_) {
  A = A_;
  B = B_;
}
void KalmanFilter::setMeasurementMatrix(const Eigen::MatrixXd &H_) { H = H_; }

void KalmanFilter::predict() { predict(Eigen::VectorXd()); }
void KalmanFilter::predict(const Eigen::VectorXd &input) {
  X = A * X;
  if (B.size() != 0 && input.size() != 0) {
    X += B * input;
  }
  P = A * P * A.transpose();
  if (Q.size() != 0) {
    P += Q;
  }
}
void KalmanFilter::update(const Eigen::VectorXd &measurements) {
  Eigen::MatrixXd S = H * P * H.transpose();
  if (R.size() != 0) {
    S += R;
  }
  auto K = P * H.transpose() * S.inverse();
  X = X + K * (measurements - H * X);
  Eigen::MatrixXd I(X.size(), X.size());
  I.setIdentity();
  P = (I - K * H) * P;
}

void ExtendedKalmanFilter::setStateUpdateFunction(
    state_function_t stateUpdateFunction_) {
  stateFunction = stateUpdateFunction_;
}
void ExtendedKalmanFilter::setMeasurementFunction(
    measurement_function_t measurementFunction_) {
  measurementFunction = measurementFunction_;
}
void ExtendedKalmanFilter::setStateJacobian(
    state_jacobian_function_t functionThatReturnsF_) {
  stateJacobian = functionThatReturnsF_;
}
void ExtendedKalmanFilter::setMeasurementJacobian(
    measurement_jacobian_function_t functionThatReturnsH_) {
  measurementJacobian = functionThatReturnsH_;
}
void ExtendedKalmanFilter::predict() { predict(Eigen::VectorXd()); }
void ExtendedKalmanFilter::predict(const Eigen::VectorXd &input) {
  X = stateFunction(X, input, userData);
  auto F = stateJacobian(X, input, userData);
  P = F * P * F.transpose();
  if (Q.size() != 0) {
    P += Q;
  }
}
void ExtendedKalmanFilter::update(const Eigen::VectorXd &measurements) {
  auto zEst = measurementFunction(X, userData);
  auto H = measurementJacobian(measurements, userData);
  Eigen::MatrixXd S = H * P * H.transpose();
  if (R.size() != 0) {
    S += R;
  }
  auto K = P * H.transpose() * S.inverse();
  X = X + K * (measurements - zEst);
  Eigen::MatrixXd I(X.size(), X.size());
  I.setIdentity();
  P = (I - K * H) * P;
}
