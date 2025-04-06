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
  const std::string &senderName = sender->GetClientName();
  const Vector3 &senderPosition = sender->GetClientPosition();

  message.sender = senderName;

  for (size_t i = 0; i < clients.size(); i++) {
    const auto &receiver = clients[i];
    if (receiver == sender) {
      continue;
    }
    if (communicationIsFeasible(receiver, senderPosition)) {
      messages[i].push_back(message);
    }
  }
}

void Broker::DispatchMessages() {
  for (size_t i = 0; i < messages.size(); i++) {
    for (size_t j = 0; j < messages[i].size(); j++) {
      clients[i]->OnMessage(messages[i][j]);
    }
    messages[i].clear();
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
