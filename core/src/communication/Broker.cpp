#include "Broker.hpp"

#include <iostream>

Broker::Broker() {}

Broker::~Broker() {}

void Broker::PrintClientsPositions() {
  for (auto client : this->clients) {
    Vector3 position = client->GetClientPosition();
    std::cout << "Client Position: " << position.x << " " << position.y << " "
              << position.z << std::endl;
  }
}
