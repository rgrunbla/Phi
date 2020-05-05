find_package(PkgConfig)

# ZeroMQ
pkg_check_modules(PC_ZeroMQ QUIET zmq)

find_path(ZeroMQ_INCLUDE_DIR
        NAMES zmq.hpp
        PATHS ${PC_ZeroMQ_INCLUDE_DIRS}
        )

find_library(ZeroMQ_LIBRARY
        NAMES zmq
        PATHS ${PC_ZeroMQ_LIBRARY_DIRS}
        )


#include(FetchContent)

#FetchContent_Declare(
#    libzmq
#    GIT_REPOSITORY https://github.com/zeromq/libzmq.git
#    EXCLUDE_FROM_ALL
#)

#FetchContent_Declare(
#    cppzmq
#    GIT_REPOSITORY https://github.com/zeromq/cppzmq.git
#    EXCLUDE_FROM_ALL
#)

#FetchContent_GetProperties(libzmq)
#if(NOT libzmq_POPULATED)
#    FetchContent_Populate(libzmq)
#    add_subdirectory(${libzmq_SOURCE_DIR} ${libzmq_BINARY_DIR})
#endif()


#FetchContent_GetProperties(cppzmq)
#if(NOT cppzmq_POPULATED)
#    FetchContent_Populate(cppzmq)
#    add_subdirectory(${cppzmq_SOURCE_DIR} ${cppzmq_BINARY_DIR})
#endif()