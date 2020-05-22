find_package(OpenGL REQUIRED)

if(NOT OpenGL_FOUND)
    message("OpenGL not found.")
else()
    message("OpenGL found through classic channels")
endif()