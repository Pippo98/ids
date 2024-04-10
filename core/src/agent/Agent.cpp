#include "Agent.hpp"

#include <iostream>

#include "communication/Broker.hpp"

using namespace std;

Agent::Agent(Vector3 position, Broker *broker) {
  cout << "Creating agent" << endl;
  // Init position to center of camera
  this->position = position;
  this->broker = broker;

  this->broker->RegisterClient(this);
}

Agent::~Agent() {}

void Agent::Step() { this->position.x += 0.1; }
