#pragma once

#include <string>

#include "communication/Message.hpp"
#include "raylib.h"

/**
 * Interface for the communication client
 * This interface is used to define the communication methods
 * between the broker and the clients
 */

class ICommunicationClient {
 public:
  virtual ~ICommunicationClient(){};

  virtual std::string GetClientName() = 0;
  virtual Vector2 GetClientPosition() = 0;

  virtual bool OnMessage(Message &) = 0;
};
