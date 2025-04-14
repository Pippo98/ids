#pragma once

#include <Eigen/Core>
#include <map>

#include "communication/Clients.hpp"
#include "communication/Message.hpp"
#include "controller/ControlModel.hpp"
#include "kflib/src/kf.hpp"
#include "kflib/src/ukf.hpp"
#include "position/PositionModel.hpp"
#include "voronoi/Voronoi.hpp"

struct AgentsPositions {
  std::string name;
  size_t voronoiId;
  UnscentedKalmanFilter kf;
  bool isTarget;
  Vector2 getPosition2D() const {
    const auto &state = kf.getState();
    return Vector2{(float)state(0), (float)state(2)};
  }
  Vector3 getPosition3D() const {
    const auto &state = kf.getState();
    return Vector3{(float)state(0), (float)state(2), (float)state(4)};
  }
};

/**
 * Agent class
 * This class represents an agent in the simulation
 * It is responsible for moving around the world and communicating with other
 * agents
 * It computes its target position based on the positions of other agents using
 * Voronoi Cells
 */

class Map;
class Broker;

class Agent : public ICommunicationClient {
 public:
  using PositionModel = PositionModelXY;
  /**
   * Constructor of Agent
   * @param Vector3: Initial position
   * @param Broker: Broker instance to communicate with other actors
   */

  Agent(PositionModel initialPosition, Map &map, std::string name,
        Broker *broker);

  /**
   * Internal step function
   * Called every step of the simulation
   * Can be realtime or faster
   *
   * TODO: Understand how can we reproduce the simulation time indipendent
   */
  void Step(float dt);

  /*************************
   * Communication Methods *
   *************************/

  /**
   * Sends the current position of the agent to the broker
   *
   * @param Vector2 position: The current position of the agent
   */
  void BroadcastPosition();

  /**
   * Receives the position of all other agents from the broker
   * This function can be called only by a Broker instance
   *
   * @param Vector3 Pointer: List of positions of all the agents in the world
   */
  bool OnMessage(Message &message) override;

  /***********************
   * Voronoi Methods     *
   ***********************/

  void SolveVoronoi();

  void DrawVoronoi() { this->solver.draw(); };

  /***********************
   * Voronoi Methods     *
   ***********************/

  void UpdateMap();

  /*************************
   * Visualization Methods *
   *************************/

  void Draw();

  /***********************
   * Getters and Setters *
   ***********************/

  PositionModel GetPosition() const { return this->position; };

  double GetWatchRadius() const { return this->watchRadius; };

  // Interface GetPosition implementation
  Vector2 GetClientPosition() override { return GetPosition(); };

  std::string GetClientName() override { return this->name; };

  const Map &GetMap() const { return posMap; }

 private:
  std::string name;
  size_t myVoronoiID;

  float stepDT;
  double lastTime = 0;
  double watchRadius = 80;

  PositionModel position;
  PositionModel targetPosition;
  ControlModelVoronoi<PositionModel> controlModel;

  bool agreement = false;
  bool isAgreeing = false;
  bool isOnTarget = false;
  double lastUpdateTime = 0;

  Map posMap;
  Broker *broker;  // Used to communicate with other actors in World
  VoronoiSolver solver;

  std::map<std::string, AgentsPositions> agentsPositions;

 public:
  /**
   * Destructor Definition
   */
  ~Agent();
};
