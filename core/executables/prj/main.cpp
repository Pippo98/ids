#include "main.hpp"

#include <stdio.h>

#include "raylib.h"

int main(void) {
  printf("IDS project\n");

  const int screenWidth = 800;
  const int screenHeight = 450;

  InitWindow(screenWidth, screenHeight, "IDS");

  Rectangle player = {400, 280, 40, 40};
  SetTargetFPS(60);
  Camera2D camera;
  camera.target = (Vector2){player.x + 20.0f, player.y + 20.0f};
  camera.offset = (Vector2){screenWidth / 2.0f, screenHeight / 2.0f};
  camera.rotation = 0.0f;
  camera.zoom = 1.0f;

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

    BeginDrawing();

    ClearBackground(RAYWHITE);
    BeginMode2D(camera);
    DrawLine(-screenWidth * 10, (int)camera.target.y, screenWidth * 10,
             (int)camera.target.y, GREEN);

    for (float i = 0; i < 1; i += 0.1) {
      for (float j = 0; j < 10; j += 0.1) {
        DrawEllipse(i * screenWidth, j * screenHeight, 10.0, 10.0, DARKGREEN);
      }
    }

    EndMode2D();

    EndDrawing();
  }

  return 0;
}
