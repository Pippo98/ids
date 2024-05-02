#pragma once

#include "kf_base.hpp"

typedef Eigen::VectorXd (*state_function_t)(const Eigen::VectorXd &state,
                                            const Eigen::VectorXd &input,
                                            void *userData);
typedef Eigen::VectorXd (*measurement_function_t)(const Eigen::VectorXd &state,
                                                  void *userData);
typedef Eigen::MatrixXd (*state_jacobian_function_t)(
    const Eigen::VectorXd &state, const Eigen::VectorXd &input, void *userData);
typedef Eigen::MatrixXd (*measurement_jacobian_function_t)(
    const Eigen::VectorXd &state, void *userData);

// Extended Kalman Filter
class ExtendedKalmanFilter : public KalmanFilterBase {
 public:
  void setStateUpdateFunction(state_function_t stateUpdateFunction);
  void setMeasurementFunction(measurement_function_t measurementFuction);
  void setStateJacobian(state_jacobian_function_t functionThatReturnsF);
  void setMeasurementJacobian(
      measurement_jacobian_function_t functionThatReturnsH);

  void predict() override;
  void predict(const Eigen::VectorXd &input) override;
  void update(const Eigen::VectorXd &measurements) override;

 private:
  state_function_t stateFunction;
  measurement_function_t measurementFunction;

  state_jacobian_function_t stateJacobian;
  measurement_jacobian_function_t measurementJacobian;
};
