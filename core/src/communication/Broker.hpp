#pragma once

#include <queue>
#include <vector>

#include "Clients.hpp"

struct ClientPosition {
  std::string name;
  Vector3 position;
};

class Broker {
 public:
  Broker() = default;
  ~Broker() = default;
  Broker(Broker &&) = default;
  Broker(const Broker &) = default;
  Broker &operator=(Broker &&) = default;
  Broker &operator=(const Broker &) = default;

  /**
   * Register a new client (Agent) to the broker
   * @param ICommunicationClient: The client to register
   */
  void RegisterClient(ICommunicationClient *client) {
    clients.push_back(client);
    clientsMessages.resize(clients.size());
  }

  void EnqueBroadcastMessage(ICommunicationClient *sender, Message &message);
  void DispatchMessages();

  void Draw() const;

 private:
  // List of registered clients
  std::vector<ICommunicationClient *> clients;
  std::vector<std::vector<Message>> clientsMessages;
};
