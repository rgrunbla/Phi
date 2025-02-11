
#ifndef MANAGER_H
#define MANAGER_H

#include "agent.h"
#include "simulation.h"
#include "zmq.hpp"
#include <glm/gtx/string_cast.hpp>
#include <helpers.h>
#include <list>
#include <math.h>
#include <memory>
#include <ns3-zmq-messages/zmq-propagation-messages.pb.h>
#include <zmq.hpp>

template <typename T> class Manager {
public:
  Manager(map_type);
  void loop();
  int32_t get_new_id();

  void dispatch_meta(std::string);
  void dispatch_meso(std::string);

  /* Create a new simulation */
  void create_simulation(std::string);
  void end_simulation(std::string);

private:
  /* ZeroMQ */
  std::shared_ptr<zmq::socket_t> zmq_socket;
  std::shared_ptr<zmq::socket_t> zmq_visualization_socket;
  std::shared_ptr<zmq::context_t> zmq_context;

  /* Simulation counter */
  int32_t counter = 0;

  map_type map;

  std::vector<std::shared_ptr<Simulation>> simulations;
};

template <typename T> Manager<T>::Manager(map_type map) {
  /* Init the Zeromq things */
  this->zmq_context = std::shared_ptr<zmq::context_t>(new zmq::context_t());
  this->zmq_socket = std::shared_ptr<zmq::socket_t>(
      new zmq::socket_t(*(this->zmq_context), zmq::socket_type::rep));
  this->zmq_socket->bind("ipc:///tmp/PhiEndpoint");

  if (VIZ) {
    this->zmq_visualization_socket = std::shared_ptr<zmq::socket_t>(
        new zmq::socket_t(*(this->zmq_context), zmq::socket_type::pub));
    this->zmq_visualization_socket->connect("ipc:///tmp/1");
  }

  this->map = map;
}

template <typename T> int32_t Manager<T>::get_new_id() {
  int32_t counter = this->counter;
  this->counter += 1;
  return counter;
}

template <typename T> void Manager<T>::dispatch_meta(std::string meta_message) {
  phi::Meta meta;
  meta.ParseFromString(meta_message);

  switch (meta.type()) {
  case phi::Meta_MessageType_INIT_QUERY:
    this->create_simulation(meta.content());
    break;
  case phi::Meta_MessageType_END_QUERY:
    this->end_simulation(meta.content());
    break;
  default:
    std::cout << "Unknown or illegal meta type !\n";
  }
}

template <typename T> void Manager<T>::dispatch_meso(std::string meso_message) {
  phi::Meso meso = phi::Meso();
  meso.ParseFromString(meso_message);
  this->simulations[meso.simulation_id()]->handleMeso(meso.type(),
                                                      meso.content());
}

template <typename T> void Manager<T>::loop() {
  phi::GlobalContainer global_container = phi::GlobalContainer();
  while (true) {
    zmq::pollitem_t items[] = {
        {static_cast<void *>(*(this->zmq_socket)), 0, ZMQ_POLLIN, 0}};
    zmq::poll(&items[0], 1, 100);
    if (items[0].revents & ZMQ_POLLIN) {
      global_container.Clear();
      global_container.ParseFromString(s_recv(*(this->zmq_socket)));
      switch (global_container.type()) {
      case phi::GlobalContainer_MessageType_META:
        this->dispatch_meta(global_container.content());
        break;

      case phi::GlobalContainer_MessageType_MESO:
        this->dispatch_meso(global_container.content());
        break;

      default:
        std::cout << "Unknown global type !\n";
      }
    }
  }
}

template <typename T>
void Manager<T>::create_simulation(std::string meta_content) {
  phi::InitAnswer init_answer = phi::InitAnswer();
  /* A new simulation in NS-3 has been launched, and ask for a simulation ID */
  std::shared_ptr<T> simulation = std::shared_ptr<T>(new T());

  simulation->zmq_socket = this->zmq_socket;
  simulation->zmq_visualization_socket = this->zmq_visualization_socket;
  simulation->set_id(this->get_new_id());
  simulation->set_map(this->map);
  simulation->set_meta(meta_content);
  simulation->init();
  simulation->start();
  this->simulations.push_back(simulation);

  init_answer.set_simulation_id(simulation->get_id());
  MetaSend(init_answer, phi::Meta_MessageType_INIT_ANSWER, *this->zmq_socket);
}

template <typename T>
void Manager<T>::end_simulation(std::string meta_content) {
  phi::EndQuery end_query = phi::EndQuery();
  std::string message;
  end_query.Clear();
  end_query.ParseFromString(meta_content);
  std::cout << "End of the simulation " << end_query.simulation_id() << "\n";

  /* Position and orientations */
  phi::AgentInfos infos = phi::AgentInfos();
  for (std::shared_ptr<Agent> agent :
       this->simulations[end_query.simulation_id()]->agents) {
    phi::AgentInfo *info = infos.add_infos();
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
    for (auto const &[name, value] : agent->generic_store) {
      phi::Value *val = info->add_values();
      value->serialize(val);
    }

    for (auto const &[name, map] : agent->generic_map_store) {
      if (map->size() != 0) {
        phi::Map *mapval = info->add_maps();
        map->serialize(mapval);
      }
    }
  }
  this->simulations[end_query.simulation_id()]->end();
  MesoSend(end_query.simulation_id(), infos, phi::Meso_MessageType_POSITIONS,
           *this->zmq_socket);
}

#endif // MANAGER_H
