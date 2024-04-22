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

  Map map((Vector2){-200, 200}, (Vector2){200, -200}, 5);
  Broker broker = Broker();
  // Init Agents

  VoronoiSolver solver = VoronoiTest();
  Voronoi &v1 = solver.addVoronoi(Vector2{0, 0}, 100);
  Agent agent = Agent(Vector3{0, 0, 0}, "Tony", &broker);
  Agent agent2 = Agent(Vector3{0, 0, 0}, "Berto", &broker);
  Agent agent3 = Agent(Vector3{0, 0, 0}, "Pippo", &broker);

  while (!WindowShouldClose()) {
    if (IsKeyDown(KEY_W))
      player.y -= 2;
    else if (IsKeyDown(KEY_S))
      player.y += 2;
    if (IsKeyDown(KEY_D))
      player.x += 2;
    else if (IsKeyDown(KEY_A))
      player.x -= 2;
    else if (IsKeyDown(KEY_Q))
      camera.zoom += 0.01;
    else if (IsKeyDown(KEY_E))
      camera.zoom -= 0.01;

    // Calculate delta time
    float deltaTime = GetFrameTime();

    camera.target.x = player.x;
    camera.target.y = player.y;
    agent.SetTargetPosition(Vector3{player.x, player.y, 0});
    agent.Step(deltaTime);
    agent2.SetTargetPosition(Vector3{-player.x, player.y, 0});
    agent2.Step(deltaTime);
    agent3.Step(deltaTime);

    // FLOW
    // agent get positions of all Agents
    // agent calculate new position
    // agent send new position to Broker
    // Broker send new positions to all Agents
    // agent update position

    BeginDrawing();

    ClearBackground(RAYWHITE);
    BeginMode2D(camera);

    if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
      Vector2 mouse = Vector2Subtract(GetMousePosition(), camera.offset);
      map.setConfidence(mouse, map.getConfidence(mouse) + 10);
    }
    float res = map.getResolution();
    Vector2 tl = map.getTopLeftCorner();
    Vector2 br = map.getBottomRightCorner();
    for (float x = tl.x; x < br.x; x += res) {
      for (float y = br.y; y < tl.y; y += res) {
        float conf = map.getConfidence((Vector2){x, y});
        Color green = DARKGREEN;
        green.a = conf;
        DrawRectangle(x, y, res, res, green);
      }
    }
    Vector2 dimension{br.x - tl.x, tl.y - br.y};
    DrawRectangleLines(tl.x, br.y, dimension.x, dimension.y, BLACK);

    v1.setPosition(Vector2Subtract(GetMousePosition(), camera.offset));
    // solver.solve();

    // for (const auto &cell : solver.getCells()) {
    //   cell.calculateCenterOfMass(map);
    // }
    // solver.draw();

    DrawLine(-screenWidth * 10, 0, screenWidth * 10, 0, GREEN);

    DrawRectangle(player.x, player.y, player.width, player.height, BLUE);

    DrawAgent(agent);
    DrawAgent(agent2);

    EndMode2D();

    EndDrawing();
  }

  return 0;
}

void DrawAgent(Agent &agent) {
  DrawEllipse(agent.GetPosition().x, agent.GetPosition().y, 10.0, 10.0, RED);
  agent.DrawVoronoi();
}

void RenderFrame() {
  // printf("RenderFrame\n");
}
