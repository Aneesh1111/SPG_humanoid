cmake_minimum_required(VERSION 3.10)
project(spg_cmake)

set(CMAKE_CXX_STANDARD 17)

# Enable testing
enable_testing()

include_directories(include)

find_package(Eigen3 QUIET)
if(NOT Eigen3_FOUND)
    message(FATAL_ERROR "Eigen3 not found. Please install it with: sudo apt-get install libeigen3-dev")
endif()
include_directories(${EIGEN3_INCLUDE_DIR})

file(GLOB SPG_SRC
    "src/spg/setpoint/*.cpp"
    "src/spg/setpoint/ConvertSegment.cpp"
    "src/spg/subtarget/*.cpp"
    "src/spg/subtarget/replan/*.cpp"
    "src/spg/subtarget/replan/search/*.cpp"
    "src/spg/subtarget/angle/*.cpp"
    "src/spg/target/*.cpp"
    "src/functions/*.cpp"
    "src/visualization/*.cpp"
    "src/*.cpp"
    "src/spg/Init.cpp"
)

add_library(spg STATIC ${SPG_SRC})

# ImGui/ImPlot integration
file(GLOB IMGUI_SRC
    "external/imgui/*.cpp"
    "external/imgui/backends/imgui_impl_opengl3.cpp"
    "external/imgui/backends/imgui_impl_glfw.cpp"
)
file(GLOB IMPLOT_SRC
    "external/implot/*.cpp"
)

add_library(imgui STATIC ${IMGUI_SRC} ${IMPLOT_SRC})
target_include_directories(imgui PUBLIC external/imgui external/imgui/backends external/implot)

target_include_directories(spg PUBLIC external/imgui external/imgui/backends external/implot)

add_executable(demo demo/main.cpp)
target_link_libraries(demo spg imgui glfw GL GLU)
target_include_directories(demo PUBLIC external/imgui external/imgui/backends external/implot)

add_subdirectory(test)
