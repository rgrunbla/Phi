find_package(Imgui)

if(NOT Imgui_FOUND)
    include(FetchContent)

    FetchContent_Declare(
        imgui
        GIT_REPOSITORY https://github.com/ocornut/imgui.git
        EXCLUDE_FROM_ALL
    )

    FetchContent_GetProperties(imgui)
    if(NOT imgui_POPULATED)
        FetchContent_Populate(imgui)
        file(COPY cmake/imgui/CMakeLists.txt DESTINATION ${imgui_SOURCE_DIR})
        add_subdirectory(${imgui_SOURCE_DIR} ${imgui_BINARY_DIR} EXCLUDE_FROM_ALL)
        message("IMGUI SOURCE DIR: " ${imgui_SOURCE_DIR}/examples)
        include_directories(${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/examples)
    endif()
else()
    message("Imgui found through classic channels")
endif()