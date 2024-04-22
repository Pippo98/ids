#include "Agent.hpp"

#include "communication/Broker.hpp"
#include "raymath.h"
#include "voronoi/Voronoi.hpp"

using namespace std;

Agent::Agent(Vector3 position, std::string name, Broker *broker) {
  // Init position to center of camera
  this->position = position;
  this->name = name;
  this->broker = broker;

  this->broker->RegisterClient(this);
  this->solver = VoronoiSolver();
  this->agentsVoronoi = map<string, Voronoi *>();
}

Agent::~Agent() {}

void Agent::Step(float deltaTime) {
  // If the agent is moving does not compute the target position
  if (Vector3Distance(this->position, this->targetPosition) <= 10) {
    this->shouldMove = false;
  } else {
    this->shouldMove = true;
  }

  if (this->shouldMove) {
    Move(deltaTime);
    // Update position once a second
  }

  auto time = GetTime();
  if (time - this->lastUpdateTime > 1) {
    this->lastUpdateTime = time;
    this->SendPosition();
  }

  solver = VoronoiSolver();
  solver.addVoronoi({position.x, position.y}, 100);
  for (const auto &[name, pos] : agentsPositions) {
    solver.addVoronoi({pos.x, pos.y}, 100);
  }
  this->solver.solve();
}

void Agent::SendPosition() {
  this->broker->BroadcastPosition(this->name, this->position);
}

void Agent::Move(float deltaTime) {
  this->position =
      Vector3MoveTowards(this->position, this->targetPosition, 100 * deltaTime);
}

bool Agent::OnMessage(Message message) {
  printf("Agent %s received message from %s\n", this->name.c_str(),
         message.name.c_str());

  // Update LOCAL positions
  this->agentsPositions.insert_or_assign(message.name, message.position);

  return true;
}
