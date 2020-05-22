#ifndef NODE_H
#define NODE_H

#include "event.h"
#include "simulation.h"
#include "vector_store.h"
#include "map_store.h"
#include "glm/glm.hpp"
#include "glm/ext.hpp"
#include <functional>
#include <variant>
#include "clock.h"

class Agent {
    public:
        Agent();
        virtual ~Agent() = 0;

        int32_t get_id() const;
        void set_id(int32_t);

        std::string get_type() const;
        void set_type(std::string);

        /* In unit, e.g. meters */
        glm::dvec3 get_position() const;
        void set_position(glm::dvec3);

        /* In unit/s */
        glm::dvec3 get_speed() const;
        void set_speed(glm::dvec3);

        glm::dvec3 get_acceleration() const;
        void set_acceleration(glm::dvec3);

        glm::dquat get_orientation() const;
        void set_orientation(glm::dquat);

        glm::dquat get_spin() const;
        void set_spin(glm::dquat);

        glm::dvec3 get_angular_velocity() const;
        void set_angular_velocity(glm::dvec3);

        std::shared_ptr<Clock> get_clock() const;
        void set_clock(std::shared_ptr<Clock>);

        Event start_at(double time);

        virtual void main() = 0;

        void set_schedule_event(std::function<void(Event)>);
        std::function<void(Event)> schedule_event;

        virtual void handleMessage(int32_t, std::string) = 0;


        /* Generic store */
        std::map<std::string, std::shared_ptr<VectorStore>> generic_store;
        std::map<std::string, std::shared_ptr<MapStore>> generic_map_store;

    private:
        int32_t id;

        std::string agent_type;

        std::shared_ptr<Clock> clock;

        /* Simulation Truth */
        glm::dvec3 position;
        glm::dquat orientation;

        glm::dvec3 speed;
        glm::dquat spin;
        
        glm::dvec3 acceleration;
        glm::dvec3 angular_velocity;
};

#endif // NODE_H