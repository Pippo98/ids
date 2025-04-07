// clang-format off
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"
// clang-format on

#include <stdio.h>

#include <memory>

#include "agent/Agent.hpp"
#include "communication/Broker.hpp"
#include "map/Map.hpp"
#include "raylib.h"
#include "raymath.h"
#include "voronoi/Voronoi.hpp"

void RenderFrame();
void DrawAgent(Agent &);
void HandleKeyboardInput(Rectangle &, Camera2D &);
void DrawMap(const Map &map, unsigned char maxAlpha = 255);

int main(void) {
  printf("IDS project\n");

  const int screenWidth = 1200;
  const int screenHeight = 1050;

  InitWindow(screenWidth, screenHeight, "IDS");

  Rectangle player = {0, 0, 3, 3};
  SetTargetFPS(60);
  Camera2D camera;
  camera.target = (Vector2){player.x, player.y};
  camera.offset = (Vector2){screenWidth / 2.0f, screenHeight / 2.0f};
  camera.rotation = 0.0f;
  camera.zoom = 1.0f;

  Map map((Vector2){-200, 200}, (Vector2){200, -200}, 20);
  Broker broker;
  // Init Agents

  // Craete a list of Agents
  std::vector<std::shared_ptr<Agent>> agents;
  agents.push_back(
      std::make_shared<Agent>(Vector2{10, 0}, map, "Tony", &broker));
  agents.push_back(
      std::make_shared<Agent>(Vector2{-10, 0}, map, "Berto", &broker));
  agents.push_back(
      std::make_shared<Agent>(Vector2{0, -1}, map, "Pippo", &broker));

  bool drawRealMap = true;
  std::vector<bool> displayAgents(agents.size(), true);

  while (!WindowShouldClose()) {
    HandleKeyboardInput(player, camera);
    // Calculate delta time
    float deltaTime = GetFrameTime();

    printf("Frame time: %f\n", deltaTime);

    camera.target.x = player.x;
    camera.target.y = player.y;

    // FLOW
    // agent get positions of all Agents
    // agent calculate new position
    // agent send new position to Broker
    // Broker send new positions to all Agents
    // agent update position

    BeginDrawing();
    ClearBackground(RAYWHITE);
    constexpr float w = 20.0;
    constexpr float h = 20.0;
    Rectangle position{0.0, 0.0, w, h};
    for (size_t i = 0; i < agents.size(); i++) {
      bool val = displayAgents[i];
      GuiCheckBox(position, agents[i]->GetClientName().c_str(), &val);
      displayAgents[i] = val;
      position.y += h;
    }
    position.y += h;
    GuiCheckBox(position, "Draw real map", &drawRealMap);

    BeginMode2D(camera);

    Vector2 mousePosition = Vector2Add(
        Vector2Divide(Vector2Subtract(GetMousePosition(), camera.offset),
                      Vector2{camera.zoom, camera.zoom}),
        camera.target);
    if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
      map.setConfidence(mousePosition, map.getConfidence(mousePosition) + 10);
    }

    for (auto &a : agents) {
      a->Step(deltaTime);
      map.visitLocation(*a);
    }
    broker.DispatchMessages();

    if (drawRealMap) {
      DrawMap(map);
    }
    for (size_t i = 0; i < agents.size(); i++) {
      if (displayAgents[i]) {
        DrawAgent(*(agents[i]));
      }
    }

    DrawRectangle(player.x, player.y, player.width, player.height, BLACK);

    broker.Draw();

    EndMode2D();
    EndDrawing();
  }

  return 0;
}

void DrawMap(const Map &map, unsigned char maxAlpha) {
  const float &res = map.getResolution();
  const Vector2 &tl = map.getTopLeftCorner();
  const Vector2 &br = map.getBottomRightCorner();
  for (float x = tl.x; x < br.x; x += res) {
    for (float y = br.y; y < tl.y; y += res) {
      float conf = map.getConfidence(Vector2{x, y});
      if (conf > 0) {
        auto color = Fade(DARKGREEN, conf);
        color.a = std::min(color.a, maxAlpha);
        DrawRectangle(x, y, res, res, color);
      } else if (conf < 0) {
        auto color = Fade(SKYBLUE, -2.55 * conf);
        color.a = std::min(color.a, maxAlpha);
        DrawRectangle(x, y, res, res, color);
      }
      /*if (map.getTileType(Vector2{x, y}) == TileType::VISITED) {*/
      /*  DrawRectangle(x, y, res, res, SKYBLUE);*/
      /*}*/
    }
  }
  Vector2 dimension{br.x - tl.x, tl.y - br.y};
  DrawRectangleLines(tl.x, br.y, dimension.x, dimension.y, BLACK);
}

void DrawAgent(Agent &agent) {
  DrawMap(agent.GetMap(), 50);
  DrawEllipse(agent.GetPosition().x, agent.GetPosition().y, 10.0, 10.0, RED);
  agent.DrawVoronoi();
  agent.Draw();
}

void HandleKeyboardInput(Rectangle &player, Camera2D &camera) {
  if (IsKeyDown(KEY_W))
    player.y -= 2;
  else if (IsKeyDown(KEY_S))
    player.y += 2;
  if (IsKeyDown(KEY_D))
    player.x += 2;
  else if (IsKeyDown(KEY_A))
    player.x -= 2;
  if (IsKeyDown(KEY_Q))
    camera.zoom += 0.01;
  else if (IsKeyDown(KEY_E))
    camera.zoom -= 0.01;
}

void RenderFrame() {
  // printf("RenderFrame\n");
}
