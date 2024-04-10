#include "World.hpp"

#include <agent/Agent.hpp>

World::World(int n_agents) {
  // Initialize Broker
  this->broker = Broker();
  // Initialize Agents
  for (int i = 0; i < n_agents; i++) {
    // Generate random position
    Vector3 position = Vector3(rand() % 100, rand() % 100, rand() % 100);

    Agent agent = Agent(position, &broker);
  }
  // Test comment
}

World::~World() {}
