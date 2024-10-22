#ifndef SIMULATION_H
#define SIMULATION_H

#include "clock.h"
#include "event.h"
#include <iostream>
#include <map>
#include <memory>
#include <queue>
#include <zmq.hpp>

#define VIZ 0

class Agent;

template <typename T> Agent *createInstance() { return new T; }
typedef std::map<std::string, Agent *(*)()> map_type;

class Simulation {
public:
  Simulation();
  virtual ~Simulation();

  /* Start the simulation */
  void start();
  void loop();
  void end();

  /* Environment */
  virtual void init() = 0;
  virtual void environment() = 0;

  void handleMeso(int32_t, std::string);
  virtual void handleMesoCustom(int32_t, std::string) = 0;

  void getPosition(std::string);
  void setPosition(std::string);
  void getOrientation(std::string);
  void setOrientation(std::string);

  /* Setters and getters */
  void set_id(int32_t id);
  int32_t get_id();

  void set_number_of_agents(int32_t);
  int32_t get_number_of_agents() const;

  void schedule_event(Event);
  void add_agent(std::shared_ptr<Agent>);

  void set_cursor(double);
  double get_cursor() const;

  void set_duration(double);
  double get_duration() const;

  bool ended() const;

  std::shared_ptr<Clock> get_clock();

  void set_meta(std::string);
  void set_map(map_type);
  std::shared_ptr<Agent> new_agent(std::string);

  /* Agents */
  std::vector<std::shared_ptr<Agent>> agents;

  /* Visualization */
  void plot();
  bool should_plot();

  /* ZeroMQ */
  std::shared_ptr<zmq::socket_t> zmq_socket;
  std::shared_ptr<zmq::socket_t> zmq_visualization_socket;

  std::shared_ptr<zmq::context_t> zmq_context;

private:
  /* Unique id of this simulation */
  int32_t id;

  /* Clock of the simulation */
  std::shared_ptr<Clock> clock;

  /* Duration of the simulation */
  double duration;

  /* Last plot */
  double last_plot;

  /* Events queue */
  std::priority_queue<Event, std::vector<Event>,
                      std::greater<std::vector<Event>::value_type>>
      event_queue;

  /* Number of agents in the simulation */
  int32_t number_of_agents;

  /* Has the simulation finished ? */
  bool simulation_ended = false;

  map_type map;
};

#endif // SIMULATION_H
