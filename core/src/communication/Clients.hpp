#pragma once

#include "raylib.h"

class ICommunicationClient {
public:
  virtual ~ICommunicationClient() {};
  virtual void SendPosition(Vector2) = 0;
  virtual bool OnMessage() = 0;
};
