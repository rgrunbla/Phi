#include "agent.h"
#include "event.h"
#include <functional>
#include <glm/gtx/string_cast.hpp>

#define U glm::dvec3(1.0, 0.0, 0.0)
#define V glm::dvec3(0.0, 1.0, 0.0)
#define W glm::dvec3(0.0, 0.0, 1.0)

Agent::Agent() {
    position = glm::dvec3(0.0, 0.0, 0.0);
    speed = glm::dvec3(0.0, 0.0, 0.0);
    acceleration = glm::dvec3(0.0, 0.0, 0.0);
    orientation = glm::dquat(glm::dvec3(0.0, 0.0, 0.0));
    orientation = glm::dquat(glm::dvec3(0.0, 0.0, 0.0));
}

void Agent::set_id(int32_t id) {
    this->id = id;
}

int32_t Agent::get_id() const {
    return this->id;
}

void Agent::set_type(std::string agent_type) {
    this->agent_type = agent_type;
}

std::string Agent::get_type() const {
    return this->agent_type;
}

glm::dvec3 Agent::get_position() const {
    return this->position;
}

void Agent::set_position(glm::dvec3 position) {
    this->position = position;
}

glm::dquat Agent::get_orientation() const {
    return this->orientation;
}

void Agent::set_orientation(glm::dquat orientation) {
    this->orientation = orientation;
}

glm::dquat Agent::get_spin() const {
    return this->spin;
}

void Agent::set_spin(glm::dquat spin) {
    this->spin = spin;
}

glm::dvec3 Agent::get_angular_velocity() const {
    return this->angular_velocity;
}

void Agent::set_angular_velocity(glm::dvec3 angular_velocity) {
    this->angular_velocity = angular_velocity;
}

glm::dvec3 Agent::get_speed() const {
    return this->speed;
}

void Agent::set_speed(glm::dvec3 speed) {
    this->speed = speed;
}

Event Agent::start_at(double time) {
    Event event;
    event.set_agent_id(this->get_id());
    event.set_start_date(time);
    return event;
}

std::shared_ptr<Clock> Agent::get_clock() const {
    return this->clock;
}

void Agent::set_clock(std::shared_ptr<Clock> clock) {
    this->clock = clock;
}

void Agent::main() {
    std::cout << "Default Agent main function.\n";
}

Agent::~Agent() {
}

void Agent::set_schedule_event(std::function <void(Event)> f) {
    this->schedule_event = f;
}

void Agent::handleMessage(int32_t message_type, std::string message_content) {
    std::cout << "Default handleMessage function.\n";
}