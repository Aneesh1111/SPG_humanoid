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

# Find qpOASES (optional - for HumanoidMPC)
find_path(QPOASES_INCLUDE_DIR qpOASES.hpp
    PATHS
    /usr/include
    /usr/local/include
    $ENV{HOME}/qpOASES/include
)

find_library(QPOASES_LIBRARY qpOASES
    PATHS
    /usr/lib
    /usr/local/lib
    $ENV{HOME}/qpOASES/lib
)

if(QPOASES_INCLUDE_DIR AND QPOASES_LIBRARY)
    set(QPOASES_FOUND TRUE)
    include_directories(${QPOASES_INCLUDE_DIR})
    message(STATUS "qpOASES found: ${QPOASES_LIBRARY}")
    add_definitions(-DHAVE_QPOASES)
else()
    set(QPOASES_FOUND FALSE)
    message(WARNING "qpOASES not found. HumanoidMPC will not be compiled.")
    message(WARNING "To install qpOASES:")
    message(WARNING "  git clone https://github.com/coin-or/qpOASES.git")
    message(WARNING "  cd qpOASES && mkdir build && cd build")
    message(WARNING "  cmake .. && make && sudo make install")
endif()

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
    # "src/spg/NextSample.cpp"  # Disabled - has pre-existing compilation errors
)

# Exclude HumanoidMPC if qpOASES not found
if(NOT QPOASES_FOUND)
    list(FILTER SPG_SRC EXCLUDE REGEX ".*HumanoidMPC\\.cpp$")
endif()

add_library(spg STATIC ${SPG_SRC})

# Link qpOASES if found
if(QPOASES_FOUND)
    target_link_libraries(spg ${QPOASES_LIBRARY})
endif()

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

# Add HumanoidMPC demonstration executable
add_executable(demo_humanoid_mpc demo/demo_humanoid_mpc.cpp)
target_link_libraries(demo_humanoid_mpc spg imgui glfw GL GLU)
target_include_directories(demo_humanoid_mpc PUBLIC external/imgui external/imgui/backends external/implot)

# Add rotation visualization test demo
add_executable(demo_rotation_test demo/demo_rotation_test.cpp)
target_link_libraries(demo_rotation_test spg imgui glfw GL GLU)
target_include_directories(demo_rotation_test PUBLIC external/imgui external/imgui/backends external/implot)

add_subdirectory(test)
