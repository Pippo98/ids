#pragma once

#include <raylib.h>

#include <string>

struct AgentPosition {
  Vector3 position;
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

Message CreatePositionMessage(Vector3, bool);
