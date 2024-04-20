#include <raylib.h>
#include <stdio.h>

#include "agent/Agent.hpp"
#include "communication/Broker.hpp"
#include "map/Map.hpp"
#include "raylib.h"
#include "raymath.h"
#include "voronoi/Voronoi.hpp"

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
  Agent agent = Agent(Vector3(), &broker);

  VoronoiSolver solver = VoronoiTest();
  Voronoi &v1 = solver.addVoronoi(Vector2{0, 0}, 100);

  while (!WindowShouldClose()) {
    if (IsKeyDown(KEY_W))
      player.y -= 2;
    else if (IsKeyDown(KEY_S))
      player.y += 2;
    if (IsKeyDown(KEY_D))
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
    solver.draw();
    solver.solve();

    EndMode2D();

    EndDrawing();
  }

  return 0;
}
