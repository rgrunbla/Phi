find_package(glm)

if(NOT glm_FOUND)
    include(FetchContent)

    FetchContent_Declare(
        glm
        GIT_REPOSITORY https://github.com/g-truc/glm.git
        EXCLUDE_FROM_ALL
    )

    FetchContent_GetProperties(glm)
    if(NOT glm_POPULATED)
        FetchContent_Populate(glm)
        add_subdirectory(${glm_SOURCE_DIR} ${glm_BINARY_DIR})
    endif()
else()
    message("GLM found through classic channels")
endif()