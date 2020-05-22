#include "clock.h"

Clock::Clock() {
    this->time = 0.0;
}

void Clock::set_time(double time) {
    this->time = time;
}

double Clock::get_time() const {
    return this->time;
}

void Clock::set_cursor(double cursor) {
    this->cursor = cursor;
}

double Clock::get_cursor() const {
    return this->cursor;
}