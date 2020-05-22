#ifndef EVENT_H
#define EVENT_H

#include <iostream>
#include <functional>

class Event  {
    public:
        Event();
        double get_start_date() const;
        void set_start_date(double);

        double get_duration() const;
        void set_duration(double);

        void set_agent_id(int32_t id);
        int32_t get_agent_id();

        friend bool operator < (const Event& lhs, const Event& rhs);
        friend bool operator > (const Event& lhs, const Event& rhs);

        void set_function(std::function<void(void)>);
        std::function<void(void)> execute;

    private:
        int32_t agent_id;
        double duration;
        double start_date;
};

#endif // EVENT_H