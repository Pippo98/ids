#pragma once

#include <vector>

#include "Clients.hpp"

class Broker {
 public:
  /**
   * Constructor of Broker
   */
  Broker();

  /**
   * Register a new client (Agent) to the broker
   * @param ICommunicationClient: The client to register
   */
  void RegisterClient(ICommunicationClient *client) {
    this->clients.push_back(client);
  }

  /**
   * Util function to print the positions of all the clients
   */
  void PrintClientsPositions();

 private:
  // List of registered clients
  std::vector<ICommunicationClient *> clients;

 public:
  /**
   * Default Constructors
   */
  Broker(Broker &&) = default;
  Broker(const Broker &) = default;

  /**
   * Default Operators
   */
  Broker &operator=(Broker &&) = default;
  Broker &operator=(const Broker &) = default;

  /**
   * Destructor Definition
   */
  ~Broker();
};
