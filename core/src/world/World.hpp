#pragma once

#include <vector>

#include "communication/Broker.hpp"

class World {
 public:
  /**
   * Construct a new World object
   *
   * @param n_agents Number of agents to create
   */
  World(int n_agents);

 private:
  // List of agents in the World
  std::vector<class Agent> agents;

  // Broker for communication
  Broker broker;

 public:
  /**
   * Default constructors
   */
  World(World &&) = default;
  World(const World &) = default;

  /**
   * Default assignment operators
   */
  World &operator=(World &&) = default;
  World &operator=(const World &) = default;

  /**
   * Deconstruct definition
   */
  ~World();
};
