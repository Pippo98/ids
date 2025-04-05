#include "Agent.hpp"

#include "Eigen/src/Core/Matrix.h"
#include "communication/Broker.hpp"
#include "communication/Message.hpp"
#include "map/Map.hpp"
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
  Q(0, 0) = Q(2, 2) = Q(4, 4) = 0.1;   // covariance position-position
  Q(0, 1) = Q(2, 3) = Q(4, 3) = 0.01;  // covariance of position-speed
  Q(1, 1) = Q(3, 3) = Q(3, 3) = 0.01;  // covariance of speed-speed

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

Agent::Agent(Vector3 position, Map &posMap, std::string name, Broker *broker)
    : position(position), posMap(posMap), name(name), broker(broker) {
  this->broker->RegisterClient(this);
  this->solver = VoronoiSolver();
  this->myVoronoiID =
      solver.addVoronoi((Vector2){position.x, position.y}, watchRadius);
}

Agent::~Agent() {}

void Agent::Step(float deltaTime) {
  stepDT = deltaTime;

  // Update position every X seconds
  if (GetTime() - lastTime > 1) {
    lastTime = GetTime();
    MoveRandomly();
  }

  if ((Vector3Distance(this->position, this->targetPosition) <= 10)) {
    this->isOnTarget = true;
  } else {
    this->isOnTarget = false;
  }

  if (isOnTarget) {
    bool allOnTarget = false;
    for (const auto &[name, pos] : agentsPositions) {
      allOnTarget = allOnTarget && pos.isTarget;
    }

    if (allOnTarget) {
      isAgreeing = true;

      Message message;
      message.type = Message::AGREEMENT;
      message.data.startAgreementProcess = true;
      this->broker->BroadcastMessage(this, message);
    }
  } else {
    Move();
  }

  SolveVoronoi();
  UpdateMap();

  if (isAgreeing) {
    // Calculate next target pos with Voronoi
    Vector3 nextPos = {0, 0, 0};
    // Comunicate possible next target pos to other agents
  }

  BroadcastPosition();

  // this->SetTargetPosition(
  //     {v.getLastCenterOfMass().x, v.getLastCenterOfMass().y, 0});
}

void Agent::UpdateMap() { this->posMap.visitLocation(*this); }

void Agent::Draw() { DrawCircle(targetPosition.x, targetPosition.y, 5, BLUE); }

void Agent::BroadcastPosition() {
  // TODO: Randomize intervals
  auto time = GetTime();
  if (time - this->lastUpdateTime > 1) {
    this->lastUpdateTime = time;
    Message message = CreatePositionMessage(this->position, this->isOnTarget);
    this->broker->BroadcastMessage(this, message);
  }
}

void Agent::Move() {
  this->position =
      Vector3MoveTowards(this->position, this->targetPosition, 10 * stepDT);
}
void Agent::MoveRandomly() {
  // Generate a random angle between -45 and 45 degrees
  double angle = (rand() % 90) - 45;
  moveAngle += angle * DEG2RAD;

  // Generate a fixed Vector3 with the angle displaced 50 pixel from the agent
  // position
  Vector3 newPos = this->position;
  newPos.x = newPos.x + 100 * cos(moveAngle);
  newPos.y = newPos.y + 100 * sin(moveAngle);

  // // Ensure the move is within bounds
  if (newPos.x >= posMap.tl.x && newPos.x <= posMap.br.x &&
      newPos.y >= posMap.br.y && newPos.y <= posMap.tl.y) {
    this->targetPosition = newPos;
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
        kf.KalmanFilterBase::predict();
        size_t newVoronoiID =
            solver.addVoronoi((Vector2){0.0f, 0.0f}, watchRadius);
        agentData.voronoiId = newVoronoiID;
      }
      kf.update(Eigen::Vector3d(message.data.agentPosition.position.x,
                                message.data.agentPosition.position.y,
                                message.data.agentPosition.position.z));
      solver.getVoronoi(agentData.voronoiId)
          .setPosition(agentData.getPosition2D());
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
  solver.getVoronoi(myVoronoiID).setPosition((Vector2){position.x, position.y});

  for (auto &[name, data] : agentsPositions) {
    data.kf.predict();
    solver.getVoronoi(data.voronoiId).setPosition(data.getPosition2D());
  }
  this->solver.solve();
  for (const auto &[id, cell] : solver.getCells()) {
    cell.calculateCenterOfMass(posMap);
  }
  auto com = solver.getVoronoi(myVoronoiID).getLastCenterOfMass();

  // Update target position based on center of Mass

  // targetPosition.x = com.x;
  // targetPosition.y = com.y;
}
