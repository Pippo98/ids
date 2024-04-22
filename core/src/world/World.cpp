#include "World.hpp"

#include <agent/Agent.hpp>
#include <cstdlib>

World::World(int n_agents) {
  // Initialize Broker
  this->broker = Broker();
  // Initialize Agents
  for (int i = 0; i < n_agents; i++) {
    // Generate random position
    Vector3 position = {};
    position.x = rand() % 100;
    position.y = rand() % 100;
    position.z = rand() % 100;
    // Vector3 position = Vector3(rand() % 100, rand() % 100, rand() % 100);

    Agent agent = Agent(position, "Pippo", &broker);
  }
  // Test comment
}

World::~World() {}
