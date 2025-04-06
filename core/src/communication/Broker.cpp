#include "Broker.hpp"

#include <raymath.h>

#include "communication/Clients.hpp"
#include "raylib.h"

bool communicationIsFeasible(ICommunicationClient *client,
                             const auto &senderPosition) {
  constexpr float distance = 150.0f;
  return Vector3Distance(client->GetClientPosition(), senderPosition) <
         distance;
}

void Broker::EnqueBroadcastMessage(ICommunicationClient *sender,
                                   Message &message) {
  const Vector3 &senderPosition = sender->GetClientPosition();

  message.sender = sender->GetClientName();

  for (size_t i = 0; i < clients.size(); i++) {
    const auto &receiver = clients[i];
    if (receiver == sender) {
      continue;
    }
    if (communicationIsFeasible(receiver, senderPosition)) {
      clientsMessages[i].push_back(message);
    }
  }
}

void Broker::DispatchMessages() {
  for (size_t i = 0; i < clientsMessages.size(); i++) {
    for (size_t j = 0; j < clientsMessages[i].size(); j++) {
      clients[i]->OnMessage(clientsMessages[i][j]);
    }
    clientsMessages[i].clear();
  }
}

void Broker::Draw() const {
  for (size_t i = 0; i < clients.size(); i++) {
    for (size_t j = i + 1; j < clients.size(); j++) {
      const auto &pos1 = clients[i]->GetClientPosition();
      const auto &pos2 = clients[j]->GetClientPosition();
      if (communicationIsFeasible(clients[j], pos1)) {
        DrawLine(pos1.x, pos1.y, pos2.x, pos2.y, DARKGREEN);
      }
    }
  }
}
