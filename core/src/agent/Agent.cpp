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
  myVoronoi = &this->solver.addVoronoi({position.x, position.y}, 100);
  this->agentsVoronoi = map<string, Voronoi *>();
}

Agent::~Agent() {}

void Agent::Step(float deltaTime) {
  // If the agent is moving does not compute the target position
  if (Vector3Distance(this->position, this->targetPosition) < 1) {
    this->shouldMove = false;
  } else {
    this->shouldMove = true;
  }

  if (this->shouldMove) {
    Move(deltaTime);
    // myVoronoi->setPosition({this->position.x, this->position.y});

    // Update position once a second
    auto time = GetTime();
    if (time - this->lastUpdateTime > 1) {
      this->lastUpdateTime = time;
      this->SendPosition();
    }

  } else {
    this->solver.solve();
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

  if (this->agentsVoronoi.find(message.name) == this->agentsVoronoi.end()) {
    printf("Updating Voronoi for %s\n", message.name.c_str());
    Voronoi &v1 =
        this->solver.addVoronoi({message.position.x, message.position.y}, 100);
    this->agentsVoronoi[message.name] = &v1;
    printf("ptr %s %p\n", message.name.c_str(),
           this->agentsVoronoi[message.name]);
  } else {
    this->agentsVoronoi.at(message.name)
        ->setPosition({message.position.x, message.position.y});
    printf("ptr %s %p\n", message.name.c_str(),
           this->agentsVoronoi[message.name]);
  }

  return true;
}
