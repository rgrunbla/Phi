#include "event.h"

Event::Event() {
    this->duration = 0.0;
    this->start_date = 0.0;
}

void Event::set_start_date(double time) {
    this->start_date = time;
}

double Event::get_start_date() const {
    return this->start_date;
}

void Event::set_duration(double duration) {
    this->duration = duration;
}

double Event::get_duration() const {
    return this->duration;
}

bool operator < (const Event& lhs, const Event& rhs)
{
    return lhs.get_start_date() < rhs.get_start_date();
}

bool operator > (const Event& lhs, const Event& rhs)
{
    return lhs.get_start_date() > rhs.get_start_date();
}

void Event::set_agent_id(int32_t agent_id) {
    this->agent_id = agent_id;
}

int32_t Event::get_agent_id() {
    return this->agent_id;
}

void Event::set_function(std::function<void(void)> f) {
    this->execute = f;
}