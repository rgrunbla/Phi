#ifndef NODES_H
#define NODES_H
#include <glm/gtx/string_cast.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/random.hpp>
#include <glm/gtx/norm.hpp>
#include <array>
#include "ns3-zmq-messages/zmq-propagation-messages.pb.h"

#define PI glm::pi<double>()

class Node {
    public:
            glm::dvec3 agent_position;
            glm::dvec3 linear_speed;
            glm::dvec3 linear_acceleration;
            glm::dquat agent_orientation;
            glm::dquat angular_speed;
            glm::dquat angular_acceleration;

            Node() {
                agent_position = glm::dvec3(glm::linearRand(0, 10), glm::linearRand(0, 10), glm::linearRand(0, 10));
                linear_speed = glm::dvec3(0.0, 0.0, 0.0);
                linear_acceleration = glm::dvec3(0.0, 0.0, 0.0);

                agent_orientation = glm::dquat(glm::dvec3(0.0, 0.0, glm::linearRand(-PI/2.0, PI/2.0)));
                angular_speed = glm::dquat(glm::dvec3(0.0, 0.0, 0.0));
                angular_acceleration = glm::dquat(glm::dvec3(0.0, 0.0, 0.0));
            }

          /*   QVector3D get_agent_position() {
                return QVector3D(this->agent_position.x, this->agent_position.y, this->agent_position.z);
            }

            QVector4D get_agent_orientation() {
                return QVector4D(this->agent_orientation.x, this->agent_orientation.y, this->agent_orientation.z, this->agent_orientation.w);
            } */

            void Parse(phi::AgentInfo node) {
                agent_position = {node.agent_position(0), node.agent_position(1), node.agent_position(2)};
                // Warning: the order is not the same in memory and in the dquat initialization !
                agent_orientation.w = node.agent_orientation(0);
                agent_orientation.x = node.agent_orientation(1);
                agent_orientation.y = node.agent_orientation(2);
                agent_orientation.z = node.agent_orientation(3);
            }

            void Print() {
                std::cout << "[" << agent_position[0] << ", " << agent_position[1] << ", " << agent_position[2] << "] \n";
            }

};

#endif // NODES_H
