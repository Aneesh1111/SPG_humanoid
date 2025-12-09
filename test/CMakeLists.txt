# CMake configuration for unit tests
cmake_minimum_required(VERSION 3.10)

# Enable testing
enable_testing()

# GoogleTest setup (download and build automatically)
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/heads/main.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Add test executable
add_executable(test_balanceXY test_balanceXY.cpp)
target_link_libraries(test_balanceXY PRIVATE spg gtest_main)
target_include_directories(test_balanceXY PRIVATE ${CMAKE_SOURCE_DIR}/include)

add_executable(test_getSegments test_getSegments.cpp)
target_link_libraries(test_getSegments PRIVATE spg gtest_main)
target_include_directories(test_getSegments PRIVATE ${CMAKE_SOURCE_DIR}/include)

# Add test executable for GetSegments functions
add_executable(test_getSegments_functions test_getSegments_functions.cpp)
target_link_libraries(test_getSegments_functions PRIVATE spg gtest_main)
target_include_directories(test_getSegments_functions PRIVATE ${CMAKE_SOURCE_DIR}/include)

# Add test executable for Traj1
add_executable(test_traj1 test_traj1.cpp)
target_link_libraries(test_traj1 PRIVATE spg gtest_main)
target_include_directories(test_traj1 PRIVATE ${CMAKE_SOURCE_DIR}/include)

# Add integration test for static obstacles
add_executable(test_static_obstacles_integration test_static_obstacles_integration.cpp)
target_link_libraries(test_static_obstacles_integration PRIVATE spg gtest_main imgui glfw GL GLU)
target_include_directories(test_static_obstacles_integration PRIVATE ${CMAKE_SOURCE_DIR}/include)

# Add straight line movement test (no obstacles)
add_executable(test_move_straight_line test_move_straight_line.cpp)
target_link_libraries(test_move_straight_line PRIVATE spg gtest_main imgui glfw GL GLU)
target_include_directories(test_move_straight_line PRIVATE ${CMAKE_SOURCE_DIR}/include)

# Add setpoint Set integration test
add_executable(test_setpoint_set test_setpoint_set.cpp)
target_link_libraries(test_setpoint_set PRIVATE spg gtest_main)
target_include_directories(test_setpoint_set PRIVATE ${CMAKE_SOURCE_DIR}/include)

# Add TrajPredict test
add_executable(test_trajpredict test_trajpredict.cpp)
target_link_libraries(test_trajpredict PRIVATE spg gtest_main)
target_include_directories(test_trajpredict PRIVATE ${CMAKE_SOURCE_DIR}/include)

# Add DetermineSetpointLimits test
add_executable(test_determine_setpoint_limits test_determine_setpoint_limits.cpp)
target_link_libraries(test_determine_setpoint_limits PRIVATE spg gtest_main)
target_include_directories(test_determine_setpoint_limits PRIVATE ${CMAKE_SOURCE_DIR}/include)

# Add Bouncing Obstacles Integration test
add_executable(test_bouncing_obstacles_integration test_bouncing_obstacles_integration.cpp)
target_link_libraries(test_bouncing_obstacles_integration PRIVATE spg gtest_main imgui glfw GL GLU)
target_include_directories(test_bouncing_obstacles_integration PRIVATE ${CMAKE_SOURCE_DIR}/include)

# Add Humanoid MPC test (only if qpOASES is available)
if(QPOASES_FOUND)
    add_executable(test_humanoid_mpc test_humanoid_mpc.cpp)
    target_link_libraries(test_humanoid_mpc PRIVATE spg gtest_main)
    target_include_directories(test_humanoid_mpc PRIVATE ${CMAKE_SOURCE_DIR}/include)
    add_test(NAME HumanoidMPCTest COMMAND test_humanoid_mpc)
else()
    message(STATUS "Skipping test_humanoid_mpc (qpOASES not found)")
endif()

add_test(NAME BalanceXYTest COMMAND test_balanceXY)
add_test(NAME GetSegmentsTest COMMAND test_getSegments)
add_test(NAME GetSegmentsFunctionsTest COMMAND test_getSegments_functions)
add_test(NAME Traj1Test COMMAND test_traj1)
add_test(NAME TrajPredictTest COMMAND test_trajpredict)
add_test(NAME StaticObstaclesIntegrationTest COMMAND test_static_obstacles_integration)
add_test(NAME MoveStraightLineTest COMMAND test_move_straight_line)
add_test(NAME SetpointSetTest COMMAND test_setpoint_set)
add_test(NAME DetermineSetpointLimitsTest COMMAND test_determine_setpoint_limits)
add_test(NAME BouncingObstaclesIntegrationTest COMMAND test_bouncing_obstacles_integration)
