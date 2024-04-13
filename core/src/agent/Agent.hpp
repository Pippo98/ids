#pragma once

#include "communication/Clients.hpp"
#include "raylib.h"

/**
 * Agent class
 * This class represents an agent in the simulation
 * It is responsible for moving around the world and communicating with other
 * agents
 * It computes its target position based on the positions of other agents using
 * Voronoi Cells
 */

class Agent : public ICommunicationClient {
 public:
  /**
   * Constructor of Agent
   * @param Vector3: Initial position
   * @param Broker: Broker instance to communicate with other actors
   */
  Agent(Vector3, class Broker *);

  /**
   * Internal step function
   * Called every step of the simulation
   * Can be realtime or faster
   *
   * TODO: Understand how can we reproduce the simulation time indipendent
   */
  void Step();

  /*************************
   * Communication Methods *
   *************************/

  /**
   * Sends the current position of the agent to the broker
   *
   * @param Vector2 position: The current position of the agent
   */
  void SendPosition(Vector3) override{};

  /**
   * Receives the position of all other agents from the broker
   * This function can be called only by a Broker instance
   *
   * @param Vector3 Pointer: List of positions of all the agents in the world
   */
  bool OnMessage(Vector3 *) override { return true; };

  /***********************
   * Getters and Setters *
   ***********************/

  Vector3 GetPosition() { return this->position; };

  // Interface GetPosition implementation
  Vector3 GetClientPosition() override { return GetPosition(); };

 private:
  // Current exact position of Agent
  Vector3 position;

  // Current moving velocity of Agent
  // Vector3 velocity = {0.1, 0.1, 0.1};

  // Target position for the agent to move to
  // Vector3 targetPosition;

  // Broker instance used to communicate with other actors in World
  class Broker *broker;

 public:
  /**
   * Default Constructors
   */
  // Agent(Agent &&) = default;
  // Agent(const Agent &) = default;

  /**
   * Default Operators
   */
  // Agent &operator=(Agent &&) = default;
  // Agent &operator=(const Agent &) = default;

  /**
   * Destructor Definition
   */
  ~Agent();
};
