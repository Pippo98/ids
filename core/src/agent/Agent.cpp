#include "Agent.hpp"

#include "Eigen/src/Core/Matrix.h"
#include "communication/Broker.hpp"
#include "communication/Message.hpp"
#include "controller/ControlModel.hpp"
#include "map/Map.hpp"
#include "position/PositionModel.hpp"
#include "raylib.h"
#include "raymath.h"
#include "voronoi/Voronoi.hpp"

using namespace std;

void constructAgentUKF(UnscentedKalmanFilter &ukf) {
  auto stateUpdate = [](const Eigen::VectorXd &statePrev,
                        const Eigen::VectorXd &inputs, void *userData) {
    (void)inputs;
    double DT = *(float *)userData;
    auto state = statePrev;
    state(0) = statePrev(0) + statePrev(1) * DT;
    state(1) = statePrev(1);
    state(2) = statePrev(2) + statePrev(3) * DT;
    state(3) = statePrev(3);
    state(4) = statePrev(4) + statePrev(5) * DT;
    state(5) = statePrev(5);
    return state;
  };
  auto measurementFunction = [](const Eigen::VectorXd &state,
                                const Eigen::VectorXd &inputs, void *userData) {
    (void)inputs;
    (void)userData;
    Eigen::VectorXd measures(3);
    measures(0) = state(0);
    measures(1) = state(2);
    measures(2) = state(4);
    return measures;
  };

  Eigen::VectorXd state(6);
  state.setZero();
  // state initialization
  state(1) = state(3) = state(5) = 0.0;
  Eigen::MatrixXd P(6, 6);
  P.setIdentity();
  P *= 0.1;

  Eigen::MatrixXd Q(6, 6);
  Q.setZero();
  Q(0, 0) = Q(2, 2) = Q(4, 4) = 0.3;   // covariance position-position
  Q(0, 1) = Q(2, 3) = Q(4, 3) = 0.01;  // covariance of position-speed
  Q(1, 1) = Q(3, 3) = Q(3, 3) = 0.2;   // covariance of speed-speed

  Eigen::MatrixXd R(3, 3);
  R.setIdentity();
  R = R * 0.1;

  ukf.setState(state);
  ukf.setStateCovariance(P);
  ukf.setProcessCovariance(Q);
  ukf.setMeasurementCovariance(R);
  ukf.setStateUpdateFunction(stateUpdate);
  ukf.setMeasurementFunction(measurementFunction);
}

Agent::Agent(PositionModel position, Map &posMap, std::string name,
             Broker *broker)
    : name(name), position(position), posMap(posMap), broker(broker) {
  this->broker->RegisterClient(this);
  this->myVoronoiID = solver.addVoronoi(position, watchRadius);
}

Agent::~Agent() {}

void Agent::Step(float deltaTime) {
  stepDT = deltaTime;

  // Update position every X seconds
  if (GetTime() - lastTime > 1) {
    lastTime = GetTime();
    /*MoveRandomly();*/
  }

  SolveVoronoi();
  const auto [controlFeasible, controlTarget] =
      controlModel.computeTargetPosition(posMap,
                                         solver.getVoronoi(myVoronoiID));
  if (controlFeasible) {
    targetPosition = controlTarget;
  }

  position.Move(targetPosition, 0.5, stepDT);

  if ((Vector2Distance(this->position, this->targetPosition) <= 10)) {
    this->isOnTarget = true;
  } else {
    this->isOnTarget = false;
  }

  if (isOnTarget) {
    bool allOnTarget = true;
    for (const auto &[name, pos] : agentsPositions) {
      allOnTarget = allOnTarget && pos.isTarget;
    }

    if (allOnTarget) {
      isAgreeing = true;

      Message message;
      message.type = Message::AGREEMENT;
      message.data.startAgreementProcess = true;
      this->broker->EnqueBroadcastMessage(this, message);
    }
  } else {
  }

  UpdateMap();

  BroadcastPosition();
}

void Agent::UpdateMap() { this->posMap.visitLocation(*this); }

void Agent::Draw() {
  DrawCircle(position.x, position.y, 4, ORANGE);
  DrawCircle(targetPosition.x, targetPosition.y, 5, BLUE);
}

void Agent::BroadcastPosition() {
  // TODO: Randomize intervals
  auto time = GetTime();
  if (time - this->lastUpdateTime > 1) {
    this->lastUpdateTime = time;
    Message message = CreatePositionMessage(this->position, this->isOnTarget);
    this->broker->EnqueBroadcastMessage(this, message);
  }
}

bool Agent::OnMessage(Message &message) {
  switch (message.type) {
    case Message::POSITION: {
      bool newAgent =
          agentsPositions.find(message.sender) == agentsPositions.end();
      auto &agentData = agentsPositions[message.sender];
      UnscentedKalmanFilter &kf = agentsPositions[message.sender].kf;

      if (newAgent) {
        agentData.name = message.sender;
        constructAgentUKF(kf);
        kf.setUserData(&stepDT);
        kf.predict();
        size_t newVoronoiID =
            solver.addVoronoi(Vector2{0.0f, 0.0f}, watchRadius);
        agentData.voronoiId = newVoronoiID;
      }

      kf.update(Eigen::Vector3d(message.data.agentPosition.position.x,
                                message.data.agentPosition.position.y, 0.0));
      return true;
    }
    case Message::AGREEMENT:
      this->isAgreeing = message.data.startAgreementProcess;
      return true;
    default:
      return false;
  }
}

void Agent::SolveVoronoi() {
  solver.getVoronoi(myVoronoiID).setPosition(position);

  for (auto &[name, data] : agentsPositions) {
    data.kf.predict();
    solver.getVoronoi(data.voronoiId).setPosition(data.getPosition2D());
  }

  this->solver.solve();

  const auto &cells = solver.getCells();
  for (size_t i = 0; i < cells.size(); i++) {
    cells[i].calculateCenterOfMass(posMap);
  }
}
