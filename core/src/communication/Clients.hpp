#pragma once

#include "raylib.h"

/**
 * Interface for the communication client
 * This interface is used to define the communication methods
 * between the broker and the clients
 */

class ICommunicationClient {
 public:
  virtual ~ICommunicationClient(){};
  virtual Vector3 GetClientPosition() = 0;
  virtual void SendPosition(Vector3) = 0;
  virtual bool OnMessage(Vector3*) = 0;
};
