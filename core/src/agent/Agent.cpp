#include "Agent.hpp"
#include <iostream>

using namespace std;

Agent::Agent(Broker *broker) {
  cout << "Creating agent" << endl;
  // Init position to center of camera
  this->position = {400, 280};
  this->broker = broker;

  this->broker->RegisterClient(this);
}

Agent::~Agent() {
}

void Agent::Step() {
  this->position.x += 0.1;
}
