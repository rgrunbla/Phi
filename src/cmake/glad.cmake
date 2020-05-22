find_package(GLAD)

if(NOT GLAD_FOUND)
    include(FetchContent)

    FetchContent_Declare(
        glad
        GIT_REPOSITORY https://github.com/Dav1dde/glad.git
        EXCLUDE_FROM_ALL
    )

    FetchContent_GetProperties(glad)
    if(NOT glad_POPULATED)
        FetchContent_Populate(glad)
        set(GLAD_INSTALL ON)
        add_subdirectory(${glad_SOURCE_DIR} ${glad_BINARY_DIR})
    endif()
else()
    message("GLAD found through classic channels")
endif()