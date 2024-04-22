#include "Broker.hpp"

#include <raymath.h>

#include <iostream>
#include <map>

#include "communication/Clients.hpp"

Broker::Broker() {}

Broker::~Broker() {}

void Broker::PrintClientsPositions() {
  for (auto client : this->clients) {
    Vector3 position = client->GetClientPosition();
    std::cout << "Client Position: " << position.x << " " << position.y << " "
              << position.z << std::endl;
  }
}

void Broker::SendMessageToClients() {
  std::map<std::string, Vector3> positions;

  for (auto client : this->clients) {
    positions[client->GetClientName()] = client->GetClientPosition();
  }

  for (auto client : this->clients) {
    // client->OnMessage(Message{positions});
  }
}

void Broker::BroadcastPosition(std::string name, Vector3 position) {
  // Possibility to filter the clients to sent the message to based on the
  // relative positions of the agents
  //
  // Filter clients based on the relative positions of the agents
  float distance = 100.0f;
  std::vector<ICommunicationClient *> filteredClients;
  std::copy_if(this->clients.begin(), this->clients.end(),
               std::back_inserter(filteredClients),
               [name, distance, position](ICommunicationClient *client) {
                 return true;

                 // Filter clients based on the distance
                 return Vector3Distance(client->GetClientPosition(), position) <
                        distance;
               });

  for (auto client : this->clients) {
    if (client->GetClientName() != name) {
      client->OnMessage(Message{name, position});
    }
  }
}

void Broker::AgreementMessage() {}
