#include <stdio.h>

#include "agent/Agent.hpp"
#include "communication/Broker.hpp"
#include "raylib.h"
#include "voronoi/Voronoi.hpp"

int main(void) {
  printf("IDS project\n");

  const int screenWidth = 1200;
  const int screenHeight = 1050;

  InitWindow(screenWidth, screenHeight, "IDS");

  Rectangle player = {400, 280, 40, 40};
  SetTargetFPS(60);
  Camera2D camera;
  camera.target = (Vector2){player.x + 20.0f, player.y + 20.0f};
  camera.offset = (Vector2){screenWidth / 2.0f, screenHeight / 2.0f};
  camera.rotation = 0.0f;
  camera.zoom = 1.5f;

  Broker broker = Broker();
  // Init Agents
  Agent agent = Agent(Vector3(), &broker);

  VoronoiSolver solved = VoronoiTest();

  while (!WindowShouldClose()) {
    if (IsKeyDown(KEY_W))
      player.y -= 2;
    else if (IsKeyDown(KEY_S))
      player.y += 2;
    else if (IsKeyDown(KEY_D))
      player.x += 2;
    else if (IsKeyDown(KEY_A))
      player.x -= 2;

    camera.target.x = player.x;
    camera.target.y = player.y;
    // agent.Step();

    BeginDrawing();

    ClearBackground(RAYWHITE);
    BeginMode2D(camera);
    // DrawLine(-screenWidth * 10, (int)camera.target.y, screenWidth * 10,
    //          (int)camera.target.y, GREEN);

    // DrawEllipse(agent.GetPosition().x, agent.GetPosition().y, 10.0, 10.0,
    // RED);
    // DrawEllipse(screenWidth / 2.0, screenHeight / 2.0, 10.0, 10.0, GREEN);
    // DrawEllipse(0, 0, 10.0, 10.0, GREEN);

    // for (float i = 0; i < 1; i += 0.1) {
    //   for (float j = 0; j < 10; j += 0.1) {
    //     DrawEllipse(i * screenWidth + 5.0, j * screenHeight
    //     + 5.0, 10.0, 10.0, DARKGREEN);
    //   }
    // }
    solved.draw();

    EndMode2D();

    EndDrawing();
  }

  return 0;
}
