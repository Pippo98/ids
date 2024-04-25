#include "Agent.hpp"

#include "communication/Broker.hpp"
#include "communication/Message.hpp"
#include "map/Map.hpp"
#include "raymath.h"
#include "voronoi/Voronoi.hpp"

using namespace std;

Agent::Agent(Vector3 position, const Map &posMap, std::string name,
             Broker *broker)
    : position(position), posMap(posMap), name(name), broker(broker) {
  this->broker->RegisterClient(this);
  this->solver = VoronoiSolver();
  this->agentsVoronoi = ::map<string, Voronoi *>();
}

Agent::~Agent() {}

void Agent::Step(float deltaTime) {
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
    Move(deltaTime);
  }

  SolveVoronoi();

  if (isAgreeing) {
    // Calculate next target pos with Voronoi
    Vector3 nextPos = {0, 0, 0};
    // Comunicate possible next target pos to other agents
  }

  BroadcastPosition();

  // this->SetTargetPosition(
  //     {v.getLastCenterOfMass().x, v.getLastCenterOfMass().y, 0});
}

void Agent::BroadcastPosition() {
  // TODO: Randomize intervals
  auto time = GetTime();
  if (time - this->lastUpdateTime > 1) {
    this->lastUpdateTime = time;
    Message message = CreatePositionMessage(this->position, this->isOnTarget);
    this->broker->BroadcastMessage(this, message);
  }
}

void Agent::Move(float deltaTime) {
  this->position =
      Vector3MoveTowards(this->position, this->targetPosition, 100 * deltaTime);
}

bool Agent::OnMessage(Message &message) {
  switch (message.type) {
    case Message::POSITION:
      this->agentsPositions.insert_or_assign(message.sender,
                                             message.data.agentPosition);
      return true;
    case Message::AGREEMENT:
      this->isAgreeing = message.data.startAgreementProcess;
      return true;
    default:
      return false;
  }
}

void Agent::SolveVoronoi() {
  solver = VoronoiSolver();
  solver.addVoronoi({position.x, position.y}, 100);

  for (const auto &[name, pos] : agentsPositions) {
    solver.addVoronoi({pos.position.x, pos.position.y}, 100);
  }
  this->solver.solve();
}
