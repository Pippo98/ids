#pragma once

#include <Eigen/Core>
#include <ostream>

#include "eigen/Eigen/src/Core/Matrix.h"

class KalmanFilterBase {
 public:
  void setUserData(void *data);

  void setState(Eigen::VectorXd state);
  void setStateCovariance(Eigen::MatrixXd stateCovariance);
  void setProcessCovariance(Eigen::MatrixXd processCovariance);
  void setMeasurementCovariance(Eigen::MatrixXd measurementCovariance);

  virtual void predict() = 0;
  virtual void predict(const Eigen::VectorXd &input) = 0;
  virtual void update(const Eigen::VectorXd &measurements) = 0;

  Eigen::VectorXd getState();
  Eigen::MatrixXd getCovariance();

  void print();
  void printToStream(std::ostream &stream);

 protected:
  void *userData;

  Eigen::VectorXd X;
  Eigen::MatrixXd P;
  Eigen::MatrixXd Q;
  Eigen::MatrixXd R;
};
