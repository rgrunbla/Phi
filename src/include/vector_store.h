#ifndef VECTOR_STORE_H
#define VECTOR_STORE_H

#include <string>
#include <iostream>
#include <string>
#include <variant>
#include <vector>
#include <phi-messages/messages.pb.h>

class VectorStore {
    public:
        VectorStore();
        VectorStore(std::string);

        void set(std::variant<std::vector<int>, std::vector<float>, std::vector<double>, std::vector<std::string>>);
        void set(std::variant<int, float, double, std::string>);
        void set(int, std::variant<int, float, double, std::string>);

        std::variant<int, float, double, std::string> get(int);

        void push_back(std::variant<int, float, double, std::string>);
        void print();
        void serialize(Value*);
        std::variant<std::vector<int>, std::vector<float>, std::vector<double>, std::vector<std::string>> values;

    private:
        std::string name;
        bool initialized = false;

};

std::ostream& operator<<(std::ostream& os, const VectorStore& store_value);

#endif // VECTOR_STORE_H