#include <iostream>
#include "vector_store.h"
#include "ns3-zmq-messages/zmq-propagation-messages.pb.h"

template<class... Ts> struct overload : Ts... { using Ts::operator()...; };
template<class... Ts> overload(Ts...) -> overload<Ts...>;

VectorStore::VectorStore() { }

VectorStore::VectorStore(std::string name) {
    this->name = name;
}

void VectorStore::set(std::variant<std::vector<int>, std::vector<float>, std::vector<double>, std::vector<std::string>> values) {
    this->values = values;
    this->initialized = true;
}

void VectorStore::set(std::variant<int, float, double, std::string> value) {
    std::visit(overload{
            [&] (int value) { this->values = std::vector<int> { value }; },
            [&] (float value) { this->values = std::vector<float> { value }; },
            [&] (double value) { this->values = std::vector<double> { value }; },
            [&] (std::string value) { this->values = std::vector<std::string> { value }; }
        }, value);
    this->initialized = true;
}

std::variant<int, float, double, std::string> VectorStore::get(int index) {
    std::variant<int, float, double, std::string> val;
    std::visit(overload{
            [&,index] (std::vector<int> value) { val = (value[index]); },
            [&,index] (std::vector<float> value) { val = (value[index]); },
            [&,index] (std::vector<double> value) { val = (value[index]); },
            [&,index] (std::vector<std::string> value) { val = (value[index]); }
        }, this->values);
    return val;
}

void VectorStore::set(int index, std::variant<int, float, double, std::string> value) {
    std::visit(overload{
        [&] (int value) { std::get<std::vector<int>>(this->values)[index] = value; },
        [&] (float value) { std::get<std::vector<float>>(this->values)[index] = value; },
        [&] (double value) { std::get<std::vector<double>>(this->values)[index] = value; },
        [&] (std::string value) { std::get<std::vector<std::string>>(this->values)[index] = value; }
    }, value);
}

void VectorStore::push_back(std::variant<int, float, double, std::string> value) {
    if(!initialized) {
        this->set(value);
    } else {
        std::visit(overload{
            [&] (int value) { std::get<std::vector<int>>(this->values).push_back(value); },
            [&] (float value) { std::get<std::vector<float>>(this->values).push_back(value); },
            [&] (double value) { std::get<std::vector<double>>(this->values).push_back(value); },
            [&] (std::string value) { std::get<std::vector<std::string>>(this->values).push_back(value); }
        }, value);
    }
}

void VectorStore::print() {
    std::visit([](auto&& values) {
        std::string delim = "";
        for(auto value: values) { std::cout << delim << value; delim = ","; }
        }, this->values);
        std::cout << "\n";
}

void VectorStore::serialize(phi::Value* value) {
    std::string message;
    value->set_name(this->name);
    std::visit(overload{
        [&value] (std::vector<int> values) {
            for(auto v: values) value->mutable_integers()->add_values(v);
            },
        [&value] (std::vector<float> values) {
            for(auto v: values) value->mutable_floats()->add_values(v);
            },
        [&value] (std::vector<double> values) {
            for(auto v: values) value->mutable_doubles()->add_values(v);
            },
        [&value] (std::vector<std::string> values) {
            for(auto v: values) value->mutable_strings()->add_values(v);
            }
    }, this->values);
}

std::ostream& operator<<(std::ostream& os, const VectorStore& store_value) {
    std::visit([&os](auto&& values) {
        std::string delim = "";
        for(auto value: values) { os << delim << value; delim = ","; }
        }, store_value.values);
    return os;
}
