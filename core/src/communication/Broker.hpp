#pragma once

#include "Clients.hpp"

class Broker {
public:
  Broker();
  Broker(Broker &&) = default;
  Broker(const Broker &) = default;
  Broker &operator=(Broker &&) = default;
  Broker &operator=(const Broker &) = default;
  ~Broker();
  void RegisterClient(ICommunicationClient *client) {
    this->client = client;
  }

private:
    ICommunicationClient *client;
};
