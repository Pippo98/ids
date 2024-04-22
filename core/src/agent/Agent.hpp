#pragma once

#include <map>

#include "communication/Clients.hpp"
#include "voronoi/Voronoi.hpp"

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

  Agent(Vector3, std::string, class Broker *);

  /**
   * Internal step function
   * Called every step of the simulation
   * Can be realtime or faster
   *
   * TODO: Understand how can we reproduce the simulation time indipendent
   */
  void Step(float);

  /*************************
   * Communication Methods *
   *************************/

  /**
   * Sends the current position of the agent to the broker
   *
   * @param Vector2 position: The current position of the agent
   */
  void SendPosition() override;

  /**
   * Receives the position of all other agents from the broker
   * This function can be called only by a Broker instance
   *
   * @param Vector3 Pointer: List of positions of all the agents in the world
   */
  bool OnMessage(Message) override;

  /***********************
   * Voronoi Methods     *
   ***********************/

  void DrawVoronoi() { this->solver.draw(); };

  /***********************
   * Getters and Setters *
   ***********************/

  Vector3 GetPosition() { return this->position; };

  double GetWatchRadius() { return this->watchRadius; };

  // Interface GetPosition implementation
  Vector3 GetClientPosition() override { return GetPosition(); };

  std::string GetClientName() override { return this->name; };

  void SetTargetPosition(Vector3 target) { this->targetPosition = target; };

 private:
  /**
   * Move the agent towards the target position
   * @param float deltaTime: The time passed since the last frame
   */
  void Move(float);

 private:
  // Current exact position of Agent
  Vector3 position;
  double watchRadius = 100;

  // Current moving velocity of Agent
  // Vector3 velocity = {0.1, 0.1, 0.1};

  // Target position for the agent to move to
  Vector3 targetPosition;
  bool shouldMove = true;
  double lastUpdateTime = 0;

  std::string name;

  // Broker instance used to communicate with other actors in World
  class Broker *broker;

  class VoronoiSolver solver;

  // Agreement status
  bool agreement = false;
  std::map<std::string, Vector3> agentsPositions;
  std::map<std::string, Voronoi *> agentsVoronoi;

 public:
  /**
   * Destructor Definition
   */
  ~Agent();
};
