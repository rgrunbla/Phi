#ifndef HELPERS_H
#define HELPERS_H

#include <zmq.hpp>
#include <math.h>
#include <deque>
#include <phi-messages/messages.pb.h>

inline static int s_sendmore (void *socket, char *string) {
    int size = zmq_send (socket, string, strlen (string), ZMQ_SNDMORE);
    return size;
}

inline static std::string s_recv (zmq::socket_t & socket, int flags = 0) {
    zmq::message_t message;
    socket.recv(&message, flags);
    return std::string(static_cast<char*>(message.data()), message.size());
}

inline static bool send(zmq::socket_t& socket, const std::string& string) {
    zmq::message_t message(string.size());
    std::memcpy (message.data(), string.data(), string.size());
    bool rc = socket.send (message, ZMQ_DONTWAIT);
    return (rc);
}

#define M_PI 3.14159265358979323846

inline static double constrainAngleRad(double x){
    x = fmod(x, (2*M_PI));
    if (x < 0)
        x += (2*M_PI);
    return x;
}

inline static double constrainAngleDeg(double x){
    x = fmod(x,360);
    if (x < 0)
        x += 360;
    return x;
}

inline static void GlobalSend(std::string message, GlobalContainer_MessageType message_type, std::shared_ptr<zmq::socket_t> zmq_socket) {
    GlobalContainer global_container = GlobalContainer();
    global_container.set_content(message);
    global_container.set_type(message_type);
    global_container.SerializeToString(&message);
    send(*(zmq_socket), message);
}

inline static void MetaSend(std::string message, Meta_MessageType message_type, std::shared_ptr<zmq::socket_t> zmq_socket) {
    Meta meta = Meta();
    meta.set_content(message);
    meta.set_type(message_type);
    meta.SerializeToString(&message);
    GlobalSend(message, GlobalContainer_MessageType_META, zmq_socket);
}

inline static void MesoSend(int simulation_id, std::string message, Meso_MessageType message_type, std::shared_ptr<zmq::socket_t> zmq_socket) {
    Meso meso = Meso();
    meso.set_content(message);
    meso.set_simulation_id(simulation_id);
    meso.set_type(message_type);
    meso.SerializeToString(&message);
    GlobalSend(message, GlobalContainer_MessageType_MESO, zmq_socket);
}

#endif // HELPERS_H