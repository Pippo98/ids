#pragma once

#include <raylib.h>

#include <string>

struct AgentPosition {
  Vector2 position;
  bool isTarget;
};

union MessageData {
  AgentPosition agentPosition;
  bool startAgreementProcess;
};

struct Message {
  std::string sender;

  enum { POSITION, AGREEMENT } type;
  MessageData data;
};

Message CreatePositionMessage(Vector2, bool);
