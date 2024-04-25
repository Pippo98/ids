#include "Broker.hpp"

#include <raymath.h>

#include "communication/Clients.hpp"

Broker::Broker() {}

Broker::~Broker() {}

std::vector<ICommunicationClient *> FilterClients(
    std::vector<ICommunicationClient *> clients, Vector3 senderPosition) {
  // Filter clients based on the relative positions of the agents
  float distance = 100.0f;

  std::vector<ICommunicationClient *> filteredClients;
  std::copy_if(clients.begin(), clients.end(),
               std::back_inserter(filteredClients),
               [distance, senderPosition](ICommunicationClient *client) {
                 return true;

                 // Filter clients based on the distance
                 return Vector3Distance(client->GetClientPosition(),
                                        senderPosition) < distance;
               });
  return filteredClients;
};

void Broker::BroadcastMessage(ICommunicationClient *client, Message &message) {
  const std::string &name = client->GetClientName();
  const Vector3 &position = client->GetClientPosition();

  // Set the sender of the message
  message.sender = name;

  for (auto client : FilterClients(this->clients, position)) {
    if (client->GetClientName() != name) {
      client->OnMessage(message);
    }
  }
}
