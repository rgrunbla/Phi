find_package(GLEW REQUIRED)
if (GLEW_FOUND)
        include_directories(${GLEW_INCLUDE_DIRS})
        link_libraries(${GLEW_LIBRARIES})
endif()