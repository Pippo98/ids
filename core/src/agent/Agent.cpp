#include "Agent.hpp"

#include "communication/Broker.hpp"
#include "raymath.h"

using namespace std;

Agent::Agent(Vector3 position, std::string name, Broker *broker) {
  // Init position to center of camera
  this->position = position;
  this->name = name;
  this->broker = broker;

  this->broker->RegisterClient(this);
}

Agent::~Agent() {}

void Agent::Step(float deltaTime) {
  // If the agent is moving does not compute the target position
  if (this->shouldMove) {
    Move(deltaTime);

    // Update position once a second
    auto time = GetTime();
    if (time - this->lastUpdateTime > 1) {
      this->lastUpdateTime = time;
      this->SendPosition();
    }

  } else {
    // With the given positions of the agents, compute the target position
  }
}

void Agent::SendPosition() {
  this->broker->BroadcastPosition(this->name, this->position);
}

void Agent::Move(float deltaTime) {
  this->position =
      Vector3MoveTowards(this->position, this->targetPosition, 10 * deltaTime);
}

bool Agent::OnMessage(Message message) {
  printf("Agent %s received message from %s\n", this->name.c_str(),
         message.name.c_str());

  // Update LOCAL positions
  this->agentsPositions.insert_or_assign(message.name, message.position);

  return true;
}
