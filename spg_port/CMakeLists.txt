cmake_minimum_required(VERSION 3.10)
project(spg_port VERSION 1.0 LANGUAGES CXX)

# C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# TODO MR2+: Add Eigen3 dependency
# find_package(Eigen3 REQUIRED)

# TODO MR5: Add qpOASES dependency (for HumanoidMPC)
# find_package(qpOASES REQUIRED)

# Include directories
include_directories(include)

# TODO MR2+: Add Eigen include
# include_directories(${EIGEN3_INCLUDE_DIR})

# Source files
set(SPG_SOURCES
    src/Target.cpp
    src/Subtarget.cpp
    src/Setpoint.cpp
    src/Utils.cpp
)

# TODO MR2: Add Target layer sources
# src/target/AdjustToField.cpp
# src/target/AdjustToPenaltyArea.cpp
# src/target/AdjustToGoalArea.cpp
# src/target/AdjustToObstacles.cpp
# src/target/AdjustTo3mRule.cpp

# TODO MR3: Add Subtarget layer sources
# src/subtarget/CheckCollisionFree.cpp
# src/subtarget/AngleUtils.cpp
# src/subtarget/ReplanUtils.cpp

# TODO MR4: Add Setpoint layer sources (MSL mode)
# src/setpoint/GetSegments.cpp
# src/setpoint/BalanceXY.cpp
# src/setpoint/StateCorrection.cpp
# src/setpoint/TrajectoryUtils.cpp

# TODO MR5: Add HumanoidMPC sources
# src/setpoint/HumanoidMPC.cpp

# Build library
add_library(spg_port STATIC ${SPG_SOURCES})

# TODO MR2+: Link Eigen
# target_link_libraries(spg_port Eigen3::Eigen)

# TODO MR5: Link qpOASES (for HumanoidMPC)
# target_link_libraries(spg_port qpOASES)

# Demo executable (optional)
add_executable(demo_spg demo/main.cpp)
target_link_libraries(demo_spg spg_port)

# TODO MR6: Add tests
# enable_testing()
# add_subdirectory(tests)

# Install targets
install(TARGETS spg_port DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
