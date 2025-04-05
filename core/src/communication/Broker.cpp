#include "Broker.hpp"

#include <raymath.h>

#include "communication/Clients.hpp"

bool communicationIsFeasible(ICommunicationClient *client,
                             const auto &senderPosition) {
  constexpr float distance = 100.0f;
  return Vector3Distance(client->GetClientPosition(), senderPosition) <
         distance;
}

void Broker::BroadcastMessage(ICommunicationClient *client, Message &message) {
  const std::string &name = client->GetClientName();
  const Vector3 &position = client->GetClientPosition();

  message.sender = name;

  for (const auto &client : clients) {
    if (communicationIsFeasible(client, position) &&
        client->GetClientName() != name) {
      client->OnMessage(message);
    }
  }
}
