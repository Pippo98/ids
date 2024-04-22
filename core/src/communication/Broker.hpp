#pragma once

#include <vector>

#include "Clients.hpp"

struct ClientPosition {
  std::string name;
  Vector3 position;
};

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

  void AgreementMessage();

  void SendPosition(Vector3 position, std::string name);

  /**
   * Util function to print the positions of all the clients
   */
  void PrintClientsPositions();

  void SendMessageToClients();

  void BroadcastPosition(std::string name, Vector3 position);

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
