#include <array>
#include <deque>
#include <functional>
#include <glm/gtx/string_cast.hpp>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <queue>
#include <vector>

#include "agent.h"
#include "event.h"
#include "fixed_queue.h"
#include "manager.h"
#include "math.h"
#include "simulation.h"
#include "vector_store.h"
#include <phi-messages/messages.pb.h>

class Drone : public Agent {
public:
  /* Vector of queue; each queue contains a 64 long history of the power of the
   * received frames */
  std::map<int32_t, FixedQueue<double, 64>> power_histories;
  float magnetometer = 0.0;

  void handleMessage(int32_t message_type, std::string message_content) {
    phi::MonitorQuery monitor_query = phi::MonitorQuery();

    switch (message_type) {
    case phi::Meso_MessageType_MONITOR_QUERY:
      monitor_query.ParseFromString(message_content);
      /* Push the monitor information into the node queue */
      if (power_histories.find(monitor_query.source_agent_id()) ==
          power_histories.end()) {
        FixedQueue<double, 64> queue;
        power_histories[monitor_query.source_agent_id()] = queue;
      }
      power_histories[monitor_query.source_agent_id()].push(
          monitor_query.power());

      break;
    default:
      std::cout << "Data is discarded\n";
    }
  }

  /* GPS Sensor */
  void gps_sensor(double frequency) {
    // std::cout << "[" << this->get_clock()->get_time() << "][Drone GPS " <<
    // this->get_id() << "] " << glm::to_string(this->get_position()) << ".\n";
    Event gps_sensor_event;
    auto gps_sensor_callback = std::bind(&Drone::gps_sensor, this, frequency);
    gps_sensor_event.set_function(gps_sensor_callback);
    gps_sensor_event.set_start_date(this->get_clock()->get_time() +
                                    (1.0 / frequency));
    this->schedule_event(gps_sensor_event);
  }

  /* Magnetometer */
  void magnetometer_sensor(double frequency) {
    /* We need to rename the pitch, yaw and roll because for us, UP is Z and for
     * glm, UP is Y */
    magnetometer = constrainAngleRad(glm::roll(this->get_orientation()));

    // std::cout << "[" << this->get_clock()->get_time() << "][Drone
    // Magnetometer " << this->get_id() << "] " << this->magnetometer << "\n";

    Event magnetometer_sensor_event;
    auto magnetometer_sensor_callback =
        std::bind(&Drone::magnetometer_sensor, this, frequency);
    magnetometer_sensor_event.set_function(magnetometer_sensor_callback);
    magnetometer_sensor_event.set_start_date(this->get_clock()->get_time() +
                                             (1.0 / frequency));
    this->schedule_event(magnetometer_sensor_event);
  }

  void algorithm1(int angle) {
    /* Consigne: Visualization */
    this->generic_store["angle"]->set(angle);

    /* Voisinage considéré */
    int window = std::get<int>(this->generic_store["range"]->get(0));
    int radius = window / 2;
    int min_limit = angle - radius;
    int max_limit = angle + radius;

    /* Find the first angle where we lack data */
    int new_angle = angle;
    bool missing_data = false;
    bool has_neighbors = false;
    int missing_agent = -1;

    /* We search missing data close to our current position */

    /* New */
    for (int i = 0; i < 180; ++i) {
      for (auto &q : this->power_histories) {
        has_neighbors = true;
        if (isnan(std::get<double>(this->generic_map_store["power_curves"]->get(
                q.first, constrainAngleDeg(angle + i))))) {
          new_angle = constrainAngleDeg(angle + i);
          missing_data = true;
          missing_agent = q.first;
          break;
        }
      }

      if (missing_data)
        break;

      for (auto &q : this->power_histories) {
        has_neighbors = true;
        if (isnan(std::get<double>(this->generic_map_store["power_curves"]->get(
                q.first, constrainAngleDeg(angle - i))))) {
          new_angle = constrainAngleDeg(angle - i);
          missing_data = true;
          missing_agent = q.first;
          break;
        }
      }

      if (missing_data)
        break;
    }

    if (missing_data) {
      // std::cout << "Node " << this->get_id() << " lacks data on something: "
      // << missing_agent << " at angle " << new_angle << "\n";
      this->generic_store["goal"]->set(new_angle);

      if (new_angle == angle) {
        // std::cout << "Node " << this->get_id() << " lacks data on something
        // current angle ( " << new_angle << ", " << angle << " ) with agent : "
        // << missing_agent << "\n";
        this->generic_store["timeout"]->set(
            std::get<int>(this->generic_store["timeout"]->get(0)) + 1);
      } else {
        this->generic_store["timeout"]->set(0);
      }
      if (std::get<int>(this->generic_store["timeout"]->get(0)) >=
          10) { // We spent 0.1 second at the current angle
        for (auto &q :
             this->power_histories) { // We iterate over all the agents
          if (isnan(
                  std::get<double>(this->generic_map_store["power_curves"]->get(
                      q.first, new_angle)))) { // This agent lacks data for this
                                               // angle event if we spernt some
                                               // time in the configuration
            this->generic_map_store["power_curves"]->set(q.first, angle,
                                                         -100.0);
          }
        }
        this->generic_store["timeout"]->set(0); // We reset the timeout
      }
    }

    /* Old */

    /*
                // We iterate on each node
                for(auto &q: this->power_histories) {
                    has_neighbors = true;
                    // We start from the current angle
                    for (int i=0; i<180; ++i) {
                        if(isnan(std::get<double>(this->generic_map_store["power_curves"]->get(q.first,
       constrainAngleDeg(angle+i))))) { new_angle = constrainAngleDeg(angle+i);
                            missing_data = true;
                            missing_agent = q.first;
                            break;
                        } else
       if(isnan(std::get<double>(this->generic_map_store["power_curves"]->get(q.first,
       constrainAngleDeg(angle-i))))) { new_angle = constrainAngleDeg(angle-i);
                            missing_data = true;
                            missing_agent = q.first;
                            break;
                        }
                    }
                     if(missing_data) {
                        // std::cout << "Node " << this->get_id() << " lacks
       data on something: " << missing_agent << " at angle " << new_angle <<
       "\n"; this->generic_store["goal"]->set(new_angle);

                        if(new_angle == angle) {
                            // std::cout << "Node " << this->get_id() << " lacks
       data on something current angle ( " << new_angle << ", " << angle << " )
       with agent : " << missing_agent << "\n";
                            this->generic_store["timeout"]->set(std::get<int>(this->generic_store["timeout"]->get(0))
       + 1); } else { this->generic_store["timeout"]->set(0);
                        }
                        if(std::get<int>(this->generic_store["timeout"]->get(0))
       >= 10) { // 0.5 second
                            //std::cout << "Timeout reached for angle " << angle
       << " and agent " << missing_agent << " on agent " << this->get_id() <<
       "\n"; this->generic_map_store["power_curves"]->set(missing_agent, angle,
       -100.0); this->generic_store["timeout"]->set(0);
                        }
                        break;
                    }
                } */

    if (!missing_data && has_neighbors &&
        ((std::get<int>(this->generic_store["goal"]->get(0)) == -1) ||
         std::get<int>(this->generic_store["goal"]->get(0)) == angle)) {
      // std::cout << "We have data on everything !\n";
      double max_val = -10000000;
      int max_index = angle;
      // For each angle:
      for (int i = min_limit; i < max_limit; ++i) {
        double acc = 0.0;
        for (auto &q : this->power_histories) {
          acc += std::get<double>(this->generic_map_store["power_curves"]->get(
              q.first, constrainAngleDeg(i)));
        }
        this->generic_store["power_curve"]->set(constrainAngleDeg(i), acc);
        if (acc >= max_val) {
          max_index = constrainAngleDeg(i);
          max_val = acc;
        }
      }
      new_angle = max_index;
      window = window / 2;

      this->generic_store["range"]->set(window);
      this->generic_store["goal"]->set(new_angle);
    }

    if (has_neighbors) {
      if (angle != std::get<int>(this->generic_store["goal"]->get(0))) {
        this->generic_store["last_change"]->set(this->get_clock()->get_time());
        if (std::get<int>(this->generic_store["goal"]->get(0)) > angle) {
          if ((std::get<int>(this->generic_store["goal"]->get(0)) - angle) >
              180) {
            // std::cout << "We move right\n";
            this->set_angular_velocity(glm::dvec3(0.0, 0.0, -0.50));
          } else {
            // std::cout << "We move left\n";
            this->set_angular_velocity(glm::dvec3(0.0, 0.0, 0.50));
          }
        } else {
          if ((angle - std::get<int>(this->generic_store["goal"]->get(0))) >
              180) {
            // std::cout << "We move left\n";
            this->set_angular_velocity(glm::dvec3(0.0, 0.0, 0.50));
          } else {
            // std::cout << "We move right\n";
            this->set_angular_velocity(glm::dvec3(0.0, 0.0, -0.50));
          }
        }
      } else {
        // std::cout << "Already there!\n";
        this->set_angular_velocity(glm::dvec3(0.0, 0.0, 0.0));
      }
    }

    if (!has_neighbors) {
      this->set_angular_velocity(glm::dvec3(0.0, 0.0, 0.50));
    }
  }

  /* Controller */
  void controller(double frequency) {

    // std::cout << "[" << this->get_clock()->get_time() << "][Drone " <<
    // this->get_id() << "] " << glm::to_string(this->get_position()) << "\n";

    /* Update of the data */
    float magnetometer = this->magnetometer;
    int angle = constrainAngleDeg(glm::degrees(magnetometer));
    assert(angle < 360);
    assert(angle > -1);

    /* We iterate on each node */
    for (auto &q : this->power_histories) {
      /* If there is no power curve for the node, we create one which is empty
       */

      if (this->generic_map_store["power_curves"]->find(q.first) ==
          this->generic_map_store["power_curves"]->end()) {
        this->generic_map_store["power_curves"]->set(
            q.first, std::vector<double>(360, std::nan("")));
      }

      /* We fill the power curve with the current angle. We assume the angle
       * didn't change this much since the previous execution of the controller
       */
      while (q.second.size() > 0) {
        if (isnan(std::get<double>(this->generic_map_store["power_curves"]->get(
                q.first, angle)))) {
          this->generic_map_store["power_curves"]->set(q.first, angle,
                                                       q.second.front());
        } else {
          this->generic_map_store["power_curves"]->set(
              q.first, angle,
              (std::get<double>(this->generic_map_store["power_curves"]->get(
                   q.first, angle)) +
               q.second.front()) /
                  2.0);
        }

        q.second.pop();
      }
    }

    /* Algo Perso */
    this->algorithm1(angle);
    /*
    std::cout << "Node " << this->get_id() << ":\n";
    for(auto q: this->power_curves) {
        std::cout << " -> Node " << q.first << ": ";
        for(auto v: q.second) {
            std::cout << v << ", ";
        }
        std::cout << "\n";
    }
    */

    Event event;
    auto controller_callback = std::bind(&Drone::controller, this, frequency);
    event.set_function(controller_callback);
    event.set_start_date(this->get_clock()->get_time() + (1.0 / frequency));
    this->schedule_event(event);
  }

  void main() {
    //            float antenna_gain[360]{-16.8f, -16.6f, -16.1f, -15.3f,
    //            -14.4f,-13.5f, -12.6f, -11.7f, -10.9f, -10.2f, -9.5f, -8.8f,
    //            -8.2f, -7.6f, -7.1f,-6.6f, -6.2f, -5.8f, -5.4f, -5.0f, -4.7f,
    //            -4.4f, -4.2f, -3.9f, -3.7f, -3.6f,-3.4f, -3.3f, -3.2f, -3.1f,
    //            -3.1f, -3.1f, -3.1f, -3.2f, -3.3f, -3.4f, -3.5f,-3.6f, -3.8f,
    //            -4.0f, -4.2f, -4.4f, -4.5f, -4.7f, -4.8f, -4.9f, -4.9f,
    //            -4.9f,-4.7f, -4.6f, -4.3f, -4.0f, -3.7f, -3.4f, -3.0f, -2.7f,
    //            -2.4f, -2.1f, -1.8f,-1.5f, -1.3f, -1.1f, -1.0f, -0.9f, -0.8f,
    //            -0.8f, -0.8f, -0.9f, -1.0f, -1.1f,-1.3f, -1.4f, -1.6f, -1.8f,
    //            -2.0f, -2.2f, -2.3f, -2.3f, -2.3f, -2.2f, -2.1f,-1.9f, -1.6f,
    //            -1.4f, -1.1f, -0.9f, -0.7f, -0.5f, -0.4f, -0.4f, -0.4f,
    //            -0.4f,-0.6f, -0.8f, -1.0f, -1.3f, -1.7f, -2.1f, -2.5f, -2.8f,
    //            -3.2f, -3.4f, -3.6f,-3.6f, -3.5f, -3.3f, -3.0f, -2.7f, -2.4f,
    //            -2.1f, -1.8f, -1.5f, -1.3f, -1.1f,-1.0f, -0.9f, -0.9f, -0.9f,
    //            -1.0f, -1.1f, -1.2f, -1.4f, -1.6f, -1.9f, -2.2f,-2.5f, -2.8f,
    //            -3.2f, -3.5f, -3.8f, -4.1f, -4.3f, -4.5f, -4.6f, -4.7f,
    //            -4.7f,-4.6f, -4.5f, -4.4f, -4.2f, -4.0f, -3.9f, -3.7f, -3.5f,
    //            -3.4f, -3.3f, -3.2f,-3.1f, -3.1f, -3.1f, -3.1f, -3.1f, -3.2f,
    //            -3.3f, -3.4f, -3.5f, -3.7f, -3.9f,-4.1f, -4.4f, -4.7f, -5.0f,
    //            -5.3f, -5.7f, -6.1f, -6.5f, -7.0f, -7.5f, -8.0f,-8.6f, -9.3f,
    //            -9.9f, -10.7f, -11.5f, -12.3f, -13.1f, -13.9f, -14.7f,
    //            -15.4f,-15.8f, -15.9f, -15.7f, -15.2f, -14.5f, -13.7f, -12.9f,
    //            -12.0f, -11.2f, -10.5f,-9.7f, -9.1f, -8.4f, -7.9f, -7.3f,
    //            -6.8f, -6.4f, -5.9f, -5.5f, -5.2f, -4.8f,-4.5f, -4.3f, -4.0f,
    //            -3.8f, -3.6f, -3.5f, -3.3f, -3.2f, -3.1f, -3.1f, -3.0f,-3.0f,
    //            -3.1f, -3.1f, -3.2f, -3.2f, -3.4f, -3.5f, -3.6f, -3.8f, -4.0f,
    //            -4.1f,-4.3f, -4.4f, -4.5f, -4.6f, -4.6f, -4.6f, -4.5f, -4.3f,
    //            -4.1f, -3.8f, -3.5f,-3.1f, -2.8f, -2.5f, -2.2f, -1.9f, -1.6f,
    //            -1.4f, -1.2f, -1.1f, -1.0f, -0.9f,-0.9f, -0.9f, -1.0f, -1.1f,
    //            -1.3f, -1.5f, -1.8f, -2.0f, -2.3f, -2.6f, -2.8f,-3.0f, -3.1f,
    //            -3.2f, -3.1f, -2.9f, -2.7f, -2.3f, -2.0f, -1.6f, -1.2f,
    //            -0.9f,-0.6f, -0.4f, -0.2f, -0.1f, -0.0f, 0.0f, -0.0f, -0.1f,
    //            -0.3f, -0.5f, -0.7f,-0.9f, -1.2f, -1.4f, -1.6f, -1.7f, -1.8f,
    //            -1.9f, -1.9f, -1.8f, -1.7f, -1.5f,-1.4f, -1.2f, -1.1f, -1.0f,
    //            -0.9f, -0.8f, -0.8f, -0.8f, -0.8f, -0.9f, -1.0f,-1.1f, -1.3f,
    //            -1.5f, -1.7f, -2.0f, -2.3f, -2.6f, -3.0f, -3.3f, -3.6f,
    //            -3.9f,-4.2f, -4.5f, -4.6f, -4.8f, -4.8f, -4.8f, -4.7f, -4.6f,
    //            -4.5f, -4.3f, -4.1f,-4.0f, -3.8f, -3.6f, -3.5f, -3.4f, -3.3f,
    //            -3.2f, -3.1f, -3.1f, -3.1f, -3.2f,-3.2f, -3.3f, -3.4f, -3.6f,
    //            -3.8f, -4.0f, -4.2f, -4.5f, -4.8f, -5.1f, -5.4f,-5.8f, -6.2f,
    //            -6.7f, -7.2f, -7.7f, -8.3f, -8.9f, -9.6f, -10.3f, -11.0f,
    //            -11.9f,-12.7f, -13.6f, -14.5f, -15.4f, -16.2f, -16.7f};
    float antenna_gain[360]{
        -6.8f, -6.8f, -6.8f, -6.8f, -6.7f, -6.7f, -6.7f, -6.6f, -6.6f, -6.6f,
        -6.5f, -6.5f, -6.4f, -6.4f, -6.3f, -6.3f, -6.3f, -6.2f, -6.2f, -6.1f,
        -6.1f, -6.0f, -6.0f, -5.9f, -5.9f, -5.8f, -5.7f, -5.7f, -5.6f, -5.5f,
        -5.4f, -5.3f, -5.2f, -5.1f, -5.0f, -4.9f, -4.7f, -4.6f, -4.5f, -4.4f,
        -4.3f, -4.2f, -4.1f, -4.0f, -3.9f, -3.8f, -3.7f, -3.6f, -3.5f, -3.4f,
        -3.3f, -3.1f, -3.0f, -2.8f, -2.7f, -2.5f, -2.3f, -2.2f, -2.0f, -1.8f,
        -1.6f, -1.5f, -1.3f, -1.1f, -1.0f, -0.8f, -0.7f, -0.6f, -0.5f, -0.4f,
        -0.3f, -0.2f, -0.2f, -0.1f, -0.1f, -0.1f, -0.0f, -0.0f, -0.0f, -0.0f,
        -0.0f, -0.0f, -0.0f, -0.0f, -0.0f, -0.1f, -0.1f, -0.1f, -0.1f, -0.1f,
        -0.1f, -0.1f, -0.1f, -0.1f, -0.1f, -0.1f, -0.1f, -0.1f, -0.0f, -0.0f,
        -0.0f, -0.0f, -0.0f, -0.0f, -0.0f, -0.1f, -0.1f, -0.1f, -0.2f, -0.2f,
        -0.3f, -0.4f, -0.5f, -0.6f, -0.7f, -0.8f, -0.9f, -1.0f, -1.2f, -1.3f,
        -1.5f, -1.7f, -1.8f, -2.0f, -2.2f, -2.3f, -2.5f, -2.7f, -2.9f, -3.0f,
        -3.2f, -3.3f, -3.5f, -3.6f, -3.8f, -3.9f, -4.1f, -4.2f, -4.3f, -4.4f,
        -4.5f, -4.6f, -4.7f, -4.7f, -4.8f, -4.9f, -4.9f, -5.0f, -5.1f, -5.2f,
        -5.2f, -5.3f, -5.4f, -5.5f, -5.6f, -5.6f, -5.7f, -5.8f, -5.9f, -6.0f,
        -6.1f, -6.2f, -6.3f, -6.4f, -6.4f, -6.5f, -6.6f, -6.7f, -6.7f, -6.8f,
        -6.8f, -6.9f, -6.9f, -7.0f, -7.0f, -7.0f, -7.0f, -7.0f, -7.0f, -7.0f,
        -7.0f, -7.0f, -6.9f, -6.9f, -6.9f, -6.8f, -6.8f, -6.7f, -6.7f, -6.6f,
        -6.5f, -6.5f, -6.4f, -6.3f, -6.2f, -6.2f, -6.1f, -6.0f, -6.0f, -5.9f,
        -5.8f, -5.8f, -5.7f, -5.6f, -5.6f, -5.5f, -5.5f, -5.4f, -5.3f, -5.3f,
        -5.2f, -5.1f, -5.1f, -5.0f, -4.9f, -4.9f, -4.8f, -4.7f, -4.6f, -4.6f,
        -4.5f, -4.4f, -4.3f, -4.2f, -4.1f, -4.0f, -3.8f, -3.7f, -3.6f, -3.4f,
        -3.3f, -3.1f, -3.0f, -2.8f, -2.6f, -2.4f, -2.3f, -2.1f, -1.9f, -1.7f,
        -1.6f, -1.4f, -1.2f, -1.1f, -0.9f, -0.8f, -0.7f, -0.5f, -0.4f, -0.3f,
        -0.2f, -0.2f, -0.1f, -0.1f, -0.0f, -0.0f, -0.0f, 0.0f,  -0.0f, -0.0f,
        -0.0f, -0.0f, -0.0f, -0.1f, -0.1f, -0.1f, -0.1f, -0.1f, -0.1f, -0.1f,
        -0.1f, -0.1f, -0.1f, -0.1f, -0.1f, -0.1f, -0.1f, -0.1f, -0.1f, -0.1f,
        -0.1f, -0.1f, -0.1f, -0.1f, -0.1f, -0.1f, -0.1f, -0.2f, -0.2f, -0.3f,
        -0.3f, -0.4f, -0.5f, -0.6f, -0.7f, -0.8f, -0.9f, -1.1f, -1.2f, -1.4f,
        -1.6f, -1.7f, -1.9f, -2.1f, -2.3f, -2.4f, -2.6f, -2.8f, -2.9f, -3.1f,
        -3.2f, -3.4f, -3.5f, -3.6f, -3.8f, -3.9f, -4.0f, -4.2f, -4.3f, -4.4f,
        -4.5f, -4.7f, -4.8f, -4.9f, -5.0f, -5.1f, -5.2f, -5.3f, -5.3f, -5.4f,
        -5.5f, -5.5f, -5.6f, -5.7f, -5.7f, -5.8f, -5.9f, -5.9f, -6.0f, -6.1f,
        -6.1f, -6.2f, -6.2f, -6.3f, -6.4f, -6.4f, -6.5f, -6.6f, -6.6f, -6.7f,
        -6.7f, -6.7f, -6.8f, -6.8f, -6.8f, -6.8f, -6.8f, -6.8f, -6.8f, -6.8};
    this->generic_store["antenna"] = std::make_unique<VectorStore>("antenna");
    for (int i = 0; i < 360; ++i)
      this->generic_store["antenna"]->push_back(antenna_gain[i]);

    this->generic_store["timeout"] = std::make_unique<VectorStore>("timeout");
    this->generic_store["timeout"]->set(0);

    this->generic_store["range"] = std::make_unique<VectorStore>("range");
    this->generic_store["range"]->set(360);

    this->generic_store["goal"] = std::make_unique<VectorStore>("goal");
    this->generic_store["goal"]->set(-1);

    this->generic_store["angle"] = std::make_unique<VectorStore>("angle");
    this->generic_store["last_change"] =
        std::make_unique<VectorStore>("last_change");
    this->generic_store["power_curve"] =
        std::make_unique<VectorStore>("power_curve");
    for (int i = 0; i < 360; ++i)
      this->generic_store["power_curve"]->push_back(std::nan(""));

    this->generic_map_store["power_curves"] =
        std::make_unique<MapStore>("power_curves");

    /* Set up the sensors */
    std::cout << this->get_id() << ","
              << "start"
              << ","
              << constrainAngleDeg(
                     glm::degrees(glm::roll(this->get_orientation())))
              << "\n";

    // std::cout << "Set up the sensors" << "\n";
    gps_sensor(10);
    magnetometer_sensor(100);

    /* Launch the controller */
    // std::cout << "Launch the controller" << "\n";
    controller(100);
  }

  ~Drone() {
    // std::cout << this->get_id() << "," << "end" << "," <<
    // *(this->generic_store["last_change"]) << "," <<
    // constrainAngleDeg(glm::degrees(magnetometer)) << "\n";
  }
};

class Client : public Agent {
public:
  void main() {
    // std::cout << "[" << this->get_clock()->get_time() << "][Client " <<
    // this->get_id() << "] " << glm::to_string(this->get_position()) << ".\n";

    if (this->get_clock()->get_time() >= 100) {
      return;
    }

    Event event;
    auto main_callback = std::bind(&Client::main, this);
    event.set_function(main_callback);
    event.set_start_date(this->get_clock()->get_time() + 1.0);
    this->schedule_event(event);
  }

  void handleMessage(int32_t message_type, std::string message_content) {}

  ~Client() {}
};

class DroneSimulation : public Simulation {
public:
  std::vector<std::vector<double>> gains;

  glm::dvec3 cart_to_local(std::shared_ptr<Agent> Agent1,
                           std::shared_ptr<Agent> Agent2) {
    glm::dvec3 v = Agent2->get_position() - Agent1->get_position();
    return glm::inverse(Agent1->get_orientation()) * v;
  }

  glm::dvec3 angle(std::shared_ptr<Agent> Agent1,
                   std::shared_ptr<Agent> Agent2) {
    glm::dvec3 dpos = cart_to_local(Agent1, Agent2);
    double rho = glm::length(dpos);
    double theta = glm::acos(dpos.z / rho);
    double phi = glm::atan(dpos.y, dpos.x);
    return glm::dvec3(rho, theta, phi);
  }

  double gain(std::shared_ptr<Agent> Agent1, std::shared_ptr<Agent> Agent2) {
    glm::dvec3 rtp = angle(Agent1, Agent2);
    float antenna_gain[360]{
        -6.8f, -6.8f, -6.8f, -6.8f, -6.7f, -6.7f, -6.7f, -6.6f, -6.6f, -6.6f,
        -6.5f, -6.5f, -6.4f, -6.4f, -6.3f, -6.3f, -6.3f, -6.2f, -6.2f, -6.1f,
        -6.1f, -6.0f, -6.0f, -5.9f, -5.9f, -5.8f, -5.7f, -5.7f, -5.6f, -5.5f,
        -5.4f, -5.3f, -5.2f, -5.1f, -5.0f, -4.9f, -4.7f, -4.6f, -4.5f, -4.4f,
        -4.3f, -4.2f, -4.1f, -4.0f, -3.9f, -3.8f, -3.7f, -3.6f, -3.5f, -3.4f,
        -3.3f, -3.1f, -3.0f, -2.8f, -2.7f, -2.5f, -2.3f, -2.2f, -2.0f, -1.8f,
        -1.6f, -1.5f, -1.3f, -1.1f, -1.0f, -0.8f, -0.7f, -0.6f, -0.5f, -0.4f,
        -0.3f, -0.2f, -0.2f, -0.1f, -0.1f, -0.1f, -0.0f, -0.0f, -0.0f, -0.0f,
        -0.0f, -0.0f, -0.0f, -0.0f, -0.0f, -0.1f, -0.1f, -0.1f, -0.1f, -0.1f,
        -0.1f, -0.1f, -0.1f, -0.1f, -0.1f, -0.1f, -0.1f, -0.1f, -0.0f, -0.0f,
        -0.0f, -0.0f, -0.0f, -0.0f, -0.0f, -0.1f, -0.1f, -0.1f, -0.2f, -0.2f,
        -0.3f, -0.4f, -0.5f, -0.6f, -0.7f, -0.8f, -0.9f, -1.0f, -1.2f, -1.3f,
        -1.5f, -1.7f, -1.8f, -2.0f, -2.2f, -2.3f, -2.5f, -2.7f, -2.9f, -3.0f,
        -3.2f, -3.3f, -3.5f, -3.6f, -3.8f, -3.9f, -4.1f, -4.2f, -4.3f, -4.4f,
        -4.5f, -4.6f, -4.7f, -4.7f, -4.8f, -4.9f, -4.9f, -5.0f, -5.1f, -5.2f,
        -5.2f, -5.3f, -5.4f, -5.5f, -5.6f, -5.6f, -5.7f, -5.8f, -5.9f, -6.0f,
        -6.1f, -6.2f, -6.3f, -6.4f, -6.4f, -6.5f, -6.6f, -6.7f, -6.7f, -6.8f,
        -6.8f, -6.9f, -6.9f, -7.0f, -7.0f, -7.0f, -7.0f, -7.0f, -7.0f, -7.0f,
        -7.0f, -7.0f, -6.9f, -6.9f, -6.9f, -6.8f, -6.8f, -6.7f, -6.7f, -6.6f,
        -6.5f, -6.5f, -6.4f, -6.3f, -6.2f, -6.2f, -6.1f, -6.0f, -6.0f, -5.9f,
        -5.8f, -5.8f, -5.7f, -5.6f, -5.6f, -5.5f, -5.5f, -5.4f, -5.3f, -5.3f,
        -5.2f, -5.1f, -5.1f, -5.0f, -4.9f, -4.9f, -4.8f, -4.7f, -4.6f, -4.6f,
        -4.5f, -4.4f, -4.3f, -4.2f, -4.1f, -4.0f, -3.8f, -3.7f, -3.6f, -3.4f,
        -3.3f, -3.1f, -3.0f, -2.8f, -2.6f, -2.4f, -2.3f, -2.1f, -1.9f, -1.7f,
        -1.6f, -1.4f, -1.2f, -1.1f, -0.9f, -0.8f, -0.7f, -0.5f, -0.4f, -0.3f,
        -0.2f, -0.2f, -0.1f, -0.1f, -0.0f, -0.0f, -0.0f, 0.0f,  -0.0f, -0.0f,
        -0.0f, -0.0f, -0.0f, -0.1f, -0.1f, -0.1f, -0.1f, -0.1f, -0.1f, -0.1f,
        -0.1f, -0.1f, -0.1f, -0.1f, -0.1f, -0.1f, -0.1f, -0.1f, -0.1f, -0.1f,
        -0.1f, -0.1f, -0.1f, -0.1f, -0.1f, -0.1f, -0.1f, -0.2f, -0.2f, -0.3f,
        -0.3f, -0.4f, -0.5f, -0.6f, -0.7f, -0.8f, -0.9f, -1.1f, -1.2f, -1.4f,
        -1.6f, -1.7f, -1.9f, -2.1f, -2.3f, -2.4f, -2.6f, -2.8f, -2.9f, -3.1f,
        -3.2f, -3.4f, -3.5f, -3.6f, -3.8f, -3.9f, -4.0f, -4.2f, -4.3f, -4.4f,
        -4.5f, -4.7f, -4.8f, -4.9f, -5.0f, -5.1f, -5.2f, -5.3f, -5.3f, -5.4f,
        -5.5f, -5.5f, -5.6f, -5.7f, -5.7f, -5.8f, -5.9f, -5.9f, -6.0f, -6.1f,
        -6.1f, -6.2f, -6.2f, -6.3f, -6.4f, -6.4f, -6.5f, -6.6f, -6.6f, -6.7f,
        -6.7f, -6.7f, -6.8f, -6.8f, -6.8f, -6.8f, -6.8f, -6.8f, -6.8f, -6.8};
    int index = ((rtp[2] + M_PI) * 180.0) / M_PI;
    if (index == 360)
      index--;
    if ((index < 0) || (index > 359)) {
      std::cout << "Erreur: index: " << index << "\n";
      std::cout << "rtp[2]: " << rtp[2] << ", rtp[2]+M_PI: " << (rtp[2] + M_PI)
                << " -> " << ((rtp[2] + M_PI) * 180.0) << " -> "
                << ((rtp[2] + M_PI) * 180.0) / M_PI << "\n";
      exit(-1);
    }
    return antenna_gain[index] + 4.0;
  }

  void init() {
    for (int i = 0; i < this->get_number_of_agents(); ++i) {
      std::vector<double> igains;
      for (int j = 0; j < this->get_number_of_agents(); ++j) {
        igains.push_back(0.0);
      }
      this->gains.push_back(igains);
    }
  }

  void environment() {
    for (int i = 0; i < this->get_number_of_agents(); ++i) {
      for (int j = 0; j < this->get_number_of_agents(); ++j) {
        if (i == j) {
          continue;
        }
        this->gains[i][j] = gain(this->agents[i], this->agents[j]);
      }
    }
  }

  void handleMesoCustom(int32_t meso_type, std::string meso_content) {
    switch (meso_type) {
    case phi::Meso_MessageType_LOSS_QUERY:
      this->lossQuery(meso_content);
      break;
    case phi::Meso_MessageType_MONITOR_QUERY:
      this->monitorQuery(meso_content);
      break;
    case phi::Meso_MessageType_POSITIONS:
      break;
    case phi::Meso_MessageType_END:
      break;
    default:
      std::cout << "Unknown meso type !\n";
      exit(-1);
    }
  }

  void lossQuery(std::string message) {
    phi::LossQuery loss_query = phi::LossQuery();
    phi::LossAnswer loss_answer = phi::LossAnswer();
    loss_query.ParseFromString(message);

    /* State update */
    this->set_cursor(loss_query.clock());
    this->loop();

    /* Answer creation */
    std::string outgoing_message;

    double power = 0.0;

    if (this->agents[loss_query.source_agent_id()]->get_type() == "Drone") {
      power +=
          this->gains[loss_query.source_agent_id()][loss_query.dest_agent_id()];
    }

    if (this->agents[loss_query.dest_agent_id()]->get_type() == "Drone") {
      power +=
          this->gains[loss_query.dest_agent_id()][loss_query.source_agent_id()];
    }

    loss_answer.set_power(power);

    MesoSend(this->get_id(), loss_answer,
             phi::Meso_MessageType_LOSS_ANSWER, *this->zmq_socket);
  }

  void monitorQuery(std::string message) {
    phi::MonitorQuery monitor_query = phi::MonitorQuery();
    monitor_query.ParseFromString(message);

    /* State update */
    this->set_cursor(monitor_query.clock());
    this->loop();

    /* Push the monitor information into the node queue */
    // std::cout << "[" << monitor_query.clock() << "] Received packet from node
    // " << monitor_query.source_agent_id() << " at " <<
    // monitor_query.dest_agent_id() << " at power " << monitor_query.power() <<
    // "\n";

    if (monitor_query.dest_agent_id() >= this->get_number_of_agents()) {
      std::cout << "something is fishy: received monitor query with dest agent "
                   "id bigger than the number of agent.\n";
      exit(0);
    }
    this->agents[monitor_query.dest_agent_id()]->handleMessage(
        phi::Meso_MessageType_MONITOR_QUERY, message);

    /* Answer creation */
    /* Empty message */
    SendAck(*this->zmq_socket);
  }
};

int main() {

  map_type map;
  map["Drone"] = &createInstance<Drone>;
  map["Client"] = &createInstance<Client>;

  Manager<DroneSimulation> manager = Manager<DroneSimulation>(map);
  manager.loop();
  return 0;
}
