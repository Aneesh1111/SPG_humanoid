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

# Find acados (optional - for HumanoidReferenceMPC)
find_path(ACADOS_INCLUDE_DIR acados/utils/types.h
    PATHS
    /usr/include
    /usr/local/include
    $ENV{HOME}/acados/include
    $ENV{ACADOS_SOURCE_DIR}/include
)

find_library(ACADOS_LIBRARY acados
    PATHS
    /usr/lib
    /usr/local/lib
    $ENV{HOME}/acados/lib
    $ENV{ACADOS_SOURCE_DIR}/lib
)

if(ACADOS_INCLUDE_DIR AND ACADOS_LIBRARY)
    set(ACADOS_FOUND TRUE)
    include_directories(${ACADOS_INCLUDE_DIR})
    include_directories(${ACADOS_INCLUDE_DIR}/blasfeo/include)
    include_directories(${ACADOS_INCLUDE_DIR}/hpipm/include)
    message(STATUS "acados found: ${ACADOS_LIBRARY}")
    add_definitions(-DHAVE_ACADOS)
    
    # Include generated solver path (adjust to your generated solver location)
    # The acados_template Python script will generate C code here
    if(EXISTS "${CMAKE_SOURCE_DIR}/generated/c_generated_code")
        include_directories(${CMAKE_SOURCE_DIR}/generated/c_generated_code)
        message(STATUS "acados generated code found at: ${CMAKE_SOURCE_DIR}/generated/c_generated_code")
    else()
        message(STATUS "acados generated code not found. Run acados_template to generate solver.")
    endif()
else()
    set(ACADOS_FOUND FALSE)
    message(STATUS "acados not found. HumanoidReferenceMPC will use placeholder code.")
    message(STATUS "To install acados:")
    message(STATUS "  git clone https://github.com/acados/acados.git")
    message(STATUS "  cd acados && mkdir build && cd build")
    message(STATUS "  cmake .. && make && sudo make install")
    message(STATUS "  pip install -e <acados_root>/interfaces/acados_template")
endif()

# Find qpOASES (optional - for HumanoidMPC fallback)
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
    message(STATUS "qpOASES not found. HumanoidMPC (qpOASES version) will not be compiled.")
    message(STATUS "To install qpOASES:")
    message(STATUS "  git clone https://github.com/coin-or/qpOASES.git")
    message(STATUS "  cd qpOASES && mkdir build && cd build")
    message(STATUS "  cmake .. && make && sudo make install")
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

# Exclude HumanoidReferenceMPC if acados not found (it will compile with placeholders but won't work)
# Comment this out if you want to compile with placeholder code even without acados
# if(NOT ACADOS_FOUND)
#     list(FILTER SPG_SRC EXCLUDE REGEX ".*HumanoidReferenceMPC\\.cpp$")
# endif()

add_library(spg STATIC ${SPG_SRC})

# Link qpOASES if found
if(QPOASES_FOUND)
    target_link_libraries(spg ${QPOASES_LIBRARY})
endif()

# Link acados if found
if(ACADOS_FOUND)
    target_link_libraries(spg ${ACADOS_LIBRARY})
    
    # Link generated solver library if it exists
    # The generated solver creates a shared library: libacados_ocp_solver_robot_mpc.so
    set(GENERATED_SOLVER_LIB "${CMAKE_SOURCE_DIR}/generated/c_generated_code/libacados_ocp_solver_robot_mpc.so")
    if(EXISTS "${GENERATED_SOLVER_LIB}")
        target_link_libraries(spg ${GENERATED_SOLVER_LIB})
        message(STATUS "Linking acados generated solver: ${GENERATED_SOLVER_LIB}")
    else()
        message(WARNING "acados generated solver library not found at ${GENERATED_SOLVER_LIB}")
        message(WARNING "Run: python3 scripts/generate_acados_solver.py")
    endif()
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
