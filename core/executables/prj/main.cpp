#include <stdio.h>

#include "agent/Agent.hpp"
#include "communication/Broker.hpp"
#include "raylib.h"

void RenderFrame();
void DrawAgent(Agent &);

int main(void) {
  printf("IDS project\n");

  const int screenWidth = 800;
  const int screenHeight = 450;

  InitWindow(screenWidth, screenHeight, "IDS");

  Rectangle player = {0, 0, 0, 0};
  SetTargetFPS(60);
  Camera2D camera;
  camera.target = (Vector2){player.x, player.y};
  camera.offset = (Vector2){screenWidth / 2.0f, screenHeight / 2.0f};
  camera.rotation = 0.0f;
  camera.zoom = 1.0f;

  Broker broker = Broker();
  // Init Agents
  Agent agent = Agent(Vector3{0, 0, 0}, "Tony", &broker);
  Agent agent2 = Agent(Vector3{0, 0, 0}, "Berto", &broker);
  Agent agent3 = Agent(Vector3{0, 0, 0}, "Pippo", &broker);

  while (!WindowShouldClose()) {
    if (IsKeyDown(KEY_W))
      player.y -= 2;
    else if (IsKeyDown(KEY_S))
      player.y += 2;
    else if (IsKeyDown(KEY_D))
      player.x += 2;
    else if (IsKeyDown(KEY_A))
      player.x -= 2;
    else if (IsKeyDown(KEY_Q))
      camera.zoom += 0.01;
    else if (IsKeyDown(KEY_E))
      camera.zoom -= 0.01;

    auto time = GetTime();

    // Calculate delta time
    float deltaTime = GetFrameTime();

    printf("Time: %f\n", time);

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
}

void RenderFrame() {
  // printf("RenderFrame\n");
}
