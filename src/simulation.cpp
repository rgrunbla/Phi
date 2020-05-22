#include <memory>
#include "event.h"
#include "agent.h"
#include "simulation.h"
#include "vector_store.h"
#include <helpers.h>
#include <glm/ext.hpp>
#include <glm/gtx/string_cast.hpp>
#include <phi-messages/messages.pb.h>


Simulation::Simulation() {
    this->clock = std::shared_ptr<Clock>(new Clock());
    this->clock->set_time(0.0);
    this->set_cursor(0.0);
    this->set_number_of_agents(0);
}

Simulation::~Simulation() {
}

void Simulation::set_cursor(double time) {
    this->clock->set_cursor(time);
}

double Simulation::get_cursor() const {
    return this->clock->get_cursor();
}


void Simulation::set_duration(double duration) {
    this->duration = duration;
}

double Simulation::get_duration() const {
    return this->duration;
}

void Simulation::set_id(int32_t id) {
    this->id = id;
}

int32_t Simulation::get_id() {
    return this->id;
}

void Simulation::loop() {
    this->iter = this->iter + 1;

    if(this->event_queue.empty()) {
        std::cout << "Queue of events is empty.\n";
        return;
    }

    while(true) {
        Event event = this->event_queue.top();

        if(event.get_start_date() <= this->clock->get_cursor()) {
            /* We jump to the next event (phase one) */
            double elapsed_time = event.get_start_date() - this->clock->get_time();
            this->clock->set_time(event.get_start_date());
            
            /* We execute all events that unconditionally occur between the last clock value
            and that time. This corresponds to the simulation of the environment, more specifically
            to the things that can be integrated over any period of time */

            /* Event 1: Integrate the acceleration, speed, position… */
            for(std::shared_ptr<Agent> agent: this->agents) {
                agent->set_position(agent->get_position() + agent->get_speed() * elapsed_time);
            }
            

            for(std::shared_ptr<Agent> agent: this->agents) {
                glm::dvec3 agent_angular_velocity = agent->get_angular_velocity();
                glm::dquat q = glm::dquat(0.0, agent_angular_velocity.x, agent_angular_velocity.y, agent_angular_velocity.z);
                agent->set_spin(0.5 * q * agent->get_orientation());
                agent->set_orientation(agent->get_orientation() + agent->get_spin() * elapsed_time);
            }
            

            /* Event 2: Custom events that should happen before the decisions of the agents */
            this->environment();

            /* We execute the event */
            event.execute();

            /* Remove the event from the queue */    
            this->event_queue.pop();
        } else {
            if((this->iter % 100) == 0) {
                this->plot();
                this->iter = 1;
            }
            return;
        }
    }
}

void Simulation::plot() {
    /* Send data to the GUI */
    std::string viz_message;
    Viz viz = Viz();

    /* Position and orientations */
    AgentInfos infos = AgentInfos();
    for(std::shared_ptr<Agent> agent: this->agents) {
        AgentInfo * info = infos.add_infos();
        auto position = agent->get_position();
        auto orientation = agent->get_orientation();
        info->add_agent_position(position.x);
        info->add_agent_position(position.y);
        info->add_agent_position(position.z);
        info->add_agent_orientation(orientation.w);
        info->add_agent_orientation(orientation.x);
        info->add_agent_orientation(orientation.y);
        info->add_agent_orientation(orientation.z);
        info->set_agent_type(agent->get_type());
        for(auto const& [name, value]: agent->generic_store) {
            Value * val = info->add_values();
            value->serialize(val);
        }

        for(auto const& [name, map]: agent->generic_map_store) {
            if(map->size() != 0) {
                Map * mapval = info->add_maps();
                map->serialize(mapval);
            }
        }
    }
    infos.SerializeToString(&viz_message);

    viz.set_type(Viz_MessageType_POSITIONS);
    viz.set_simulation_id(this->get_id());
    viz.set_clock(this->get_cursor());
    viz.set_content(viz_message);
    viz.SerializeToString(&viz_message);
    viz.Clear();
    s_sendmore(*(this->zmq_visualization_socket), "Viz");
    send(*(this->zmq_visualization_socket), viz_message);
}

bool Simulation::ended() const {
    return this->simulation_ended;
}

void Simulation::start() {
    this->init();
    std::cout << "Starting the simulation " << this->get_id() << "\n";
}

void Simulation::end() {
    std::cout << "Simulation " << this->get_id() << " end at " << this->clock->get_time() << "\n";
    this->simulation_ended = true;
    /* Destruct the Agents */
    for(std::shared_ptr<Agent> agent: this->agents) {
        agent->~Agent();
    }
}

void Simulation::add_agent(std::shared_ptr<Agent> agent) {
    this->agents.push_back(agent);
}

int32_t Simulation::get_number_of_agents() const {
    return this->number_of_agents;
}

void Simulation::set_number_of_agents(int32_t number_of_agents) {
    this->number_of_agents = number_of_agents;
}

void Simulation::schedule_event(Event event) {
    this->event_queue.push(event);
}

std::shared_ptr<Clock> Simulation::get_clock() {
    return this->clock;
}

void Simulation::set_map(map_type map) {
    this->map = map;
}

std::shared_ptr<Agent> Simulation::new_agent(std::string class_name) {
    std::shared_ptr<Agent> agent(this->map[class_name]());
    agent->set_type(class_name);
    return agent;
}

void Simulation::set_meta(std::string init_query_message) {
    InitQuery init_query = InitQuery();
    init_query.ParseFromString(init_query_message);
    
    this->set_number_of_agents(init_query.agent_number());

    /* Agents initialization */
    for(int a = 0; a < init_query.agent_number(); ++a) {
        AgentInfo agent_info = init_query.info(a);
        std::shared_ptr<Agent> agent = this->new_agent(agent_info.agent_type());
        agent->set_id(a);
        agent->set_clock(this->get_clock());
        agent->set_position(glm::dvec3(agent_info.agent_position(0), agent_info.agent_position(1), agent_info.agent_position(2)));
        agent->set_orientation(glm::dquat(agent_info.agent_orientation(0), agent_info.agent_orientation(1), agent_info.agent_orientation(2), agent_info.agent_orientation(3)));
        
        agent->set_angular_velocity(glm::dvec3(0.0, 0.0, 0.0));
        agent->set_speed(glm::dvec3(0.0, 0.0, 0.0));
        agent->set_spin(glm::dquat(glm::dvec3(0.0f, 0.0f, 0.0f)));

        auto schedule_event_callback = std::bind(&Simulation::schedule_event, this, std::placeholders::_1);
        agent->set_schedule_event(schedule_event_callback);
        this->add_agent(agent);
        Event event = agent->start_at(0.0);
        auto main_callback = std::bind(&Agent::main, agent);
        event.set_function(main_callback);
        this->schedule_event(event);
    }
}