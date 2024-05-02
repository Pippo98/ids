#include <Eigen/LU>
#include <iostream>
#include <ostream>
#include <unsupported/Eigen/MatrixFunctions>

#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Core/util/Meta.h"
#include "ekf.hpp"
#include "kf.hpp"
#include "kf_base.hpp"
#include "ukf.hpp"

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
    state_function_t stateFunction_) {
  stateFunction = stateFunction_;
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

UnscentedKalmanFilter::UnscentedKalmanFilter() {}

void UnscentedKalmanFilter::setStateUpdateFunction(
    state_function_t stateFunction_) {
  stateFunction = stateFunction_;
}
void UnscentedKalmanFilter::setMeasurementFunction(
    measurement_function_t measurementFunction_) {
  measurementFunction = measurementFunction_;
}

void UnscentedKalmanFilter::computeMerweScaledSigmaPoints(
    const Eigen::VectorXd &state, const Eigen::MatrixXd &P,
    MerweScaledSigmaPoints &outPoints) {
  assert(state.size() == 0);
  size_t n = state.size();
  double kappa = 3 - n;
  double lambda = sigmaPointsAlpha * sigmaPointsAlpha * (n + kappa) - n;

  double allWeightsValue = 1 / (2 * (n + lambda));

  outPoints.sigmas.resize(n, 2 * n + 1);
  outPoints.meanWeights.resize(outPoints.sigmas.size());
  outPoints.meanWeights.setConstant(allWeightsValue);
  outPoints.covarianceWeights = outPoints.meanWeights;

  outPoints.sigmas.col(0) = state;
  outPoints.meanWeights(0) = lambda / (n + lambda);
  outPoints.covarianceWeights(0) =
      lambda / (n + lambda) +
      (1 - sigmaPointsAlpha * sigmaPointsAlpha + sigmaPointsBeta);

  Eigen::MatrixXd U = ((n + lambda) * P);
  U = U.sqrt();
  for (size_t i = 0; i < n; ++i) {
    outPoints.sigmas.col(i + 1) = state - U.col(i);
    outPoints.sigmas.col(n + i + 1) = state + U.col(i);
  }
}

void UnscentedKalmanFilter::computeMeanAndCovariance(
    const MerweScaledSigmaPoints &points,
    const Eigen::MatrixXd &additionalCovariance, Eigen::VectorXd &outX,
    Eigen::MatrixXd &outP) {
  outP.resize(points.sigmas.size(), points.sigmas.size());
  outX = points.sigmas * points.meanWeights;
  outP.setZero();
  for (Eigen::Index i = 0; i < points.covarianceWeights.cols(); ++i) {
    const auto y = points.sigmas.col(i) - outX;
    outP = outP + points.covarianceWeights(i) * (y * y.transpose());
  }
  if (additionalCovariance.size() != 0) {
    outP += additionalCovariance;
  }
}

Eigen::MatrixXd UnscentedKalmanFilter::computeKalmanGain(
    const MerweScaledSigmaPoints &stateSigmaPoints,
    const MerweScaledSigmaPoints &measureSigmaPoints,
    const Eigen::VectorXd &stateEstimate,
    const Eigen::VectorXd &measureEstimate,
    const Eigen::MatrixXd &measurementCovariance) {
  size_t n = X.size();
  size_t m = measureEstimate.size();
  size_t nSigmas = stateSigmaPoints.sigmas.cols();

  Eigen::MatrixXd crossCovariance(n, m);
  for (size_t i = 0; i < nSigmas; ++i) {
    crossCovariance +=
        stateSigmaPoints.covarianceWeights(i) *
        ((stateSigmaPoints.sigmas.col(i) - stateEstimate) *
         (measureSigmaPoints.sigmas.col(i) - measureEstimate).transpose());
  }
  return crossCovariance * measurementCovariance.inverse();
}
void UnscentedKalmanFilter::predict(const Eigen::VectorXd &input) {
  computeMerweScaledSigmaPoints(X, P, stateSigmaPoints);
  for (Eigen::Index i = 0; i < stateSigmaPoints.sigmas.cols(); ++i) {
    stateSigmaPoints.sigmas.col(i) =
        stateFunction(stateSigmaPoints.sigmas.col(i), input, userData);
  }
  computeMeanAndCovariance(stateSigmaPoints, Q, X, P);
}
void UnscentedKalmanFilter::update(const Eigen::VectorXd &measurements) {
  MerweScaledSigmaPoints measureSigmaPoints;
  computeMerweScaledSigmaPoints(X, P, measureSigmaPoints);
  for (Eigen::Index i = 0; i < measureSigmaPoints.sigmas.cols(); ++i) {
    measureSigmaPoints.sigmas.col(i) =
        measurementFunction(measureSigmaPoints.sigmas.col(i), userData);
  }
  Eigen::VectorXd zEst;
  Eigen::MatrixXd Pz;
  computeMeanAndCovariance(measureSigmaPoints, R, zEst, Pz);
  auto K = computeKalmanGain(stateSigmaPoints, measureSigmaPoints, X, zEst, Pz);

  X = X + K * (measurements - zEst);
  P = P - K * Pz * K.transpose();
}
