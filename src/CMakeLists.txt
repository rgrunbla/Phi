cmake_minimum_required(VERSION 3.13)
project("Phi")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set (CMAKE_CXX_STANDARD 17)

option(BUILD_VIZ "Build the visualization toolkit" OFF)

if(BUILD_VIZ)
    include(cmake/glew.cmake)
    include(cmake/opengl.cmake)
    include(cmake/glfw.cmake)
    include(cmake/glad.cmake)
    include(cmake/imgui/imgui.cmake)
    include(cmake/imgui/imgui_plot.cmake)
endif()

include(cmake/glm.cmake)
include(cmake/zmq.cmake)
include(cmake/protobuf.cmake)

add_subdirectory(ns3-zmq-messages)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories("include")

add_executable(Phi phi.cpp simulation.cpp map_store.cpp vector_store.cpp event.cpp clock.cpp agent.cpp)
target_link_libraries(Phi glm ${ZeroMQ_LIBRARY} ns3-zmq-messages)

if(BUILD_VIZ)
    add_executable(Viz viz.cpp simulation.cpp map_store.cpp vector_store.cpp event.cpp clock.cpp agent.cpp imgui_impl_opengl3.cpp imgui_impl_glfw.cpp)
    target_link_libraries(Viz ImGui glfw ${GLFW_LIBRARIES} glad OpenGL::GL ${ZeroMQ_LIBRARY} ns3-zmq-messages imgui_plot)
endif()
