find_package(ImguiPlot)

if(NOT ImguiPlot_FOUND)
    include(FetchContent)
    FetchContent_Declare(
        imgui_plot
        GIT_REPOSITORY https://github.com/soulthreads/imgui-plot.git
        GIT_TAG v0.1.0
        EXCLUDE_FROM_ALL
    )
    FetchContent_GetProperties(imgui_plot)
    if(NOT imgui_plot_POPULATED)
        FetchContent_Populate(imgui_plot)
        add_subdirectory(${imgui_plot_SOURCE_DIR} ${imgui_plot_BINARY_DIR})
    endif()
else()
    message("ImguiPlot found through classic channels")
endif()