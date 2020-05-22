#include "agent.h"
#include "phi-messages/messages.pb.h"

Agent::Agent() {
    this->position = {0.0, 0.0, 0.0};
    this->orientation = {0.0, 0.0, 0.0, 1.0};
    this->type = "uninitialized";
    this->id = -1;
    this->angle = 0;
    this->goal = 0;
}

Agent::~Agent() {
}