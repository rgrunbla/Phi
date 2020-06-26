#ifndef MAP_STORE_H
#define MAP_STORE_H

#include <iostream>
#include <map>
#include <phi-messages/messages.pb.h>
#include <string>
#include <variant>
#include <vector>

class MapStore {
public:
  MapStore();
  MapStore(std::string);

  void set(std::variant<int, float, double, std::string>,
           std::variant<std::vector<int>, std::vector<float>,
                        std::vector<double>, std::vector<std::string>>);
  void set(std::variant<int, float, double, std::string>,
           std::variant<int, float, double, std::string>);
  void set(std::variant<int, float, double, std::string>, int,
           std::variant<int, float, double, std::string>);
  std::map<
      std::variant<int, float, double, std::string>,
      std::variant<std::vector<int>, std::vector<float>, std::vector<double>,
                   std::vector<std::string>>>::iterator
      find(std::variant<int, float, double, std::string>);
  std::map<
      std::variant<int, float, double, std::string>,
      std::variant<std::vector<int>, std::vector<float>, std::vector<double>,
                   std::vector<std::string>>>::iterator
  begin();
  std::map<
      std::variant<int, float, double, std::string>,
      std::variant<std::vector<int>, std::vector<float>, std::vector<double>,
                   std::vector<std::string>>>::iterator
  end();
  std::map<std::variant<int, float, double, std::string>,
           std::variant<std::vector<int>, std::vector<float>,
                        std::vector<double>, std::vector<std::string>>>
      values;
  int size();

  std::variant<int, float, double, std::string>
  get(std::variant<int, float, double, std::string>, int);
  void serialize(phi::Map *);
  void print();

private:
  std::string name;
  bool initialized = false;
};

// std::ostream& operator<<(std::ostream& os, const MapStore& store_value);

#endif // MAP_STORE_H