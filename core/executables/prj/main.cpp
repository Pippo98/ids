#include <raylib.h>
#include <stdio.h>

#include "agent/Agent.hpp"
#include "communication/Broker.hpp"
#include "map/Map.hpp"
#include "raylib.h"
#include "raymath.h"
#include "voronoi/Voronoi.hpp"

void RenderFrame();
void DrawAgent(Agent &);
void HandleKeyboardInput(Rectangle &, Vector3 &, Camera2D &);

int main(void) {
  printf("IDS project\n");

  const int screenWidth = 1200;
  const int screenHeight = 1050;

  InitWindow(screenWidth, screenHeight, "IDS");

  Rectangle player = {0, 0, 0, 0};
  SetTargetFPS(60);
  Camera2D camera;
  camera.target = (Vector2){player.x, player.y};
  camera.offset = (Vector2){screenWidth / 2.0f, screenHeight / 2.0f};
  camera.rotation = 0.0f;
  camera.zoom = 1.0f;

  Map map((Vector2){-200, 200}, (Vector2){200, -200}, 20);
  Broker broker;
  // Init Agents

  auto agent2pos = Vector3{0, 0, 0};
  Agent agent = Agent(Vector3{10, 0, 0}, map, "Tony", &broker);
  // Agent agent2 = Agent(agent2pos, map, "Berto", &broker);
  // Agent agent3 = Agent(Vector3{10, 0, 0}, map, "Pippo", &broker);

  // Craete a list of Agents
  std::vector<Agent *> agents;
  agents.push_back(&agent);
  // agents.push_back(&agent2);
  // agents.push_back(&agent3);

  while (!WindowShouldClose()) {
    HandleKeyboardInput(player, agent2pos, camera);
    // Calculate delta time
    float deltaTime = GetFrameTime();

    camera.target.x = player.x;
    camera.target.y = player.y;
    agent.SetTargetPosition(Vector3{player.x, player.y, 0});
    // agent2.SetTargetPosition(agent2pos);

    // FLOW
    // agent get positions of all Agents
    // agent calculate new position
    // agent send new position to Broker
    // Broker send new positions to all Agents
    // agent update position

    BeginDrawing();

    ClearBackground(RAYWHITE);
    BeginMode2D(camera);
    Vector2 mousePosition = Vector2Add(
        Vector2Divide(Vector2Subtract(GetMousePosition(), camera.offset),
                      (Vector2){camera.zoom, camera.zoom}),
        camera.target);

    if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
      map.setConfidence(mousePosition, map.getConfidence(mousePosition) + 10);
    }
    float res = map.getResolution();
    Vector2 tl = map.getTopLeftCorner();
    Vector2 br = map.getBottomRightCorner();
    for (float x = tl.x; x < br.x; x += res) {
      for (float y = br.y; y < tl.y; y += res) {
        float conf = map.getConfidence((Vector2){x, y});
        if (conf > 0) {
          Color green = DARKGREEN;
          green.a = conf;
          DrawRectangle(x, y, res, res, green);
        } else if (conf < 0) {
          Color red = RED;
          red.a = -(2.55 * conf);
          DrawRectangle(x, y, res, res, red);
        }
        if (map.getTileType((Vector2){x, y}) == TileType::VISITED)
          DrawRectangle(x, y, res, res, RED);
      }
    }
    Vector2 dimension{br.x - tl.x, tl.y - br.y};
    DrawRectangleLines(tl.x, br.y, dimension.x, dimension.y, BLACK);

    DrawLine(-screenWidth * 10, 0, screenWidth * 10, 0, GREEN);

    DrawRectangle(player.x, player.y, player.width, player.height, BLUE);

    for (auto &a : agents) {
      a->Step(deltaTime);
    }

    for (auto &a : agents) {
      DrawAgent(*a);
    }

    EndMode2D();

    EndDrawing();
  }

  return 0;
}

void DrawAgent(Agent &agent) {
  DrawEllipse(agent.GetPosition().x, agent.GetPosition().y, 10.0, 10.0, RED);
  agent.DrawVoronoi();
}

void HandleKeyboardInput(Rectangle &player, Vector3 &agent2pos,
                         Camera2D &camera) {
  if (IsKeyDown(KEY_W))
    player.y -= 2;
  else if (IsKeyDown(KEY_S))
    player.y += 2;
  if (IsKeyDown(KEY_D))
    player.x += 2;
  else if (IsKeyDown(KEY_A))
    player.x -= 2;
  if (IsKeyDown(KEY_UP)) {
    agent2pos.y -= 2;
  } else if (IsKeyDown(KEY_DOWN)) {
    agent2pos.y += 2;
  }
  if (IsKeyDown(KEY_RIGHT)) {
    agent2pos.x += 2;
  } else if (IsKeyDown(KEY_LEFT)) {
    agent2pos.x -= 2;
  }
  if (IsKeyDown(KEY_Q))
    camera.zoom += 0.01;
  else if (IsKeyDown(KEY_E))
    camera.zoom -= 0.01;
}

void RenderFrame() {
  // printf("RenderFrame\n");
}
