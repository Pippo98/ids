#include "Message.hpp"

Message CreatePositionMessage(Vector2 position, bool isTarget = false) {
  Message message;

  message.type = Message::POSITION;
  message.data.agentPosition.position = position;
  message.data.agentPosition.isTarget = isTarget;
  return message;
}
