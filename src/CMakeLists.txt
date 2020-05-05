cmake_minimum_required(VERSION 3.17)
project("Phi")

include(cmake/glm.cmake)
include(cmake/zmq.cmake)
include(cmake/protobuf.cmake)

add_subdirectory(phi-messages)
include_directories(${CMAKE_CURRENT_BINARY_DIR})
add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} glm ${ZeroMQ_LIBRARY} phi-messages)