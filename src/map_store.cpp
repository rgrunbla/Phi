#include <iostream>
#include "map_store.h"
#include "ns3-zmq-messages/zmq-propagation-messages.pb.h"

template<class... Ts> struct overload : Ts... { using Ts::operator()...; };
template<class... Ts> overload(Ts...) -> overload<Ts...>;

MapStore::MapStore() { }

MapStore::MapStore(std::string name) {
    this->name = name;
}

std::variant<int, float, double, std::string> MapStore::get(std::variant<int, float, double, std::string> key, int index) {
    std::variant<int, float, double, std::string> val;
    std::visit(overload{
            [&,index] (std::vector<int> value) { val = (value[index]); },
            [&,index] (std::vector<float> value) { val = (value[index]); },
            [&,index] (std::vector<double> value) { val = (value[index]); },
            [&,index] (std::vector<std::string> value) { val = (value[index]); }
        }, this->values[key]);

    return val;
}

void MapStore::set(std::variant<int, float, double, std::string> key, std::variant<int, float, double, std::string> value) {
        std::visit(overload{
            [&] (int value) { this->values[key] = std::vector<int> { value }; },
            [&] (float value) { this->values[key] = std::vector<float> { value }; },
            [&] (double value) { this->values[key] = std::vector<double> { value }; },
            [&] (std::string value) { this->values[key] = std::vector<std::string> { value }; }
        }, value);
    this->initialized = true;
}

void MapStore::set(std::variant<int, float, double, std::string> key, std::variant<std::vector<int>, std::vector<float>, std::vector<double>, std::vector<std::string>> values) {
    this->values[key] = values;
    this->initialized = true;
}

void MapStore::set(std::variant<int, float, double, std::string> key, int index,std::variant<int, float, double, std::string> value) {
    std::visit(overload{
        [&] (int value) { std::get<std::vector<int>>(this->values[key])[index] = value; },
        [&] (float value) { std::get<std::vector<float>>(this->values[key])[index] = value; },
        [&] (double value) { std::get<std::vector<double>>(this->values[key])[index] = value; },
        [&] (std::string value) { std::get<std::vector<std::string>>(this->values[key])[index] = value; }
    }, value);
}

std::map<std::variant<int, float, double, std::string>, std::variant<std::vector<int>, std::vector<float>, std::vector<double>, std::vector<std::string>>>::iterator MapStore::find(std::variant<int, float, double, std::string> key) {
    return this->values.find(key);
}

std::map<std::variant<int, float, double, std::string>, std::variant<std::vector<int>, std::vector<float>, std::vector<double>, std::vector<std::string>>>::iterator MapStore::begin() {
    return this->values.begin();
}

std::map<std::variant<int, float, double, std::string>, std::variant<std::vector<int>, std::vector<float>, std::vector<double>, std::vector<std::string>>>::iterator MapStore::end() {
    return this->values.end();
}

int MapStore::size() {
    return this->values.size();
}

void MapStore::serialize(phi::Map* map) {
    map->set_name(this->name);
    for( auto const& [key, val] : this->values ) {
        phi::MapValue * map_value = map->add_values();
        std::visit(overload{
            [&] (int value) { map_value->set_int_key(value); },
            [&] (float value) { map_value->set_float_key(value); },
            [&] (double value) { map_value->set_double_key(value); },
            [&] (std::string value) { map_value->set_string_key(value); }
        }, key);

        std::visit(overload{
            [&map_value] (std::vector<int> values) {
                for(auto v: values) map_value->mutable_integers()->add_values(v);
                },
            [&map_value] (std::vector<float> values) {
                for(auto v: values) map_value->mutable_floats()->add_values(v);
                },
            [&map_value] (std::vector<double> values) {
                for(auto v: values) map_value->mutable_doubles()->add_values(v);
                },
            [&map_value] (std::vector<std::string> values) {
                for(auto v: values) map_value->mutable_strings()->add_values(v);
                }
        }, val);
    }
}

void MapStore::print() {
    std::cout << "Map [" << this->name << "]:\n";
    for( auto const& [key, val] : this->values ) {
        std::visit(overload{
            [&] (std::vector<int> values) {
                for(auto v: values) std::cout << ", " << v;
                },
            [&] (std::vector<float> values) {
                for(auto v: values) std::cout << ", " << v;
                },
            [&] (std::vector<double> values) {
                for(auto v: values) std::cout << ", " << v;
                },
            [&] (std::vector<std::string> values) {
                for(auto v: values) std::cout << ", " << v;
                }
        }, val);
        std::cout << "\n";
    }
    std::cout << "End of Map.\n";
}
