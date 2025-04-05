#pragma once

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
    this->clients.push_back(client);
  }

  void BroadcastMessage(ICommunicationClient *, Message &message);

 private:
  // List of registered clients
  std::vector<ICommunicationClient *> clients;
};
