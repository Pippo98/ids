#pragma once

#include "raylib.h"
#include "communication/Clients.hpp"
#include "communication/Broker.hpp"

// Class that handles computing logic for the single agent
// Base Functions:
// - Communicate position to other actors
// - Listen for position from other actors and start compunting new target spot
// - Handle agent movement (No render)

class Agent: public ICommunicationClient {
public:
  Agent(Broker *broker);
  Agent(Agent &&) = default;
  Agent(const Agent &) = default;
  Agent &operator=(Agent &&) = default;
  Agent &operator=(const Agent &) = default;
  ~Agent();

  virtual void SendPosition(Vector2) {};

  // Function to call at every frame update
  // Used to calculate positions and communication signals
  void Step();
  Vector2 GetPosition() { return this->position; };
  bool OnMessage() { return true; };

private:
  Vector2 position;
  Vector2 target;
  Vector2 velocity = {0.1, 0.1};
  
  Broker *broker;
};

