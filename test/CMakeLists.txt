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

add_test(NAME BalanceXYTest COMMAND test_balanceXY)
add_test(NAME GetSegmentsTest COMMAND test_getSegments)
add_test(NAME GetSegmentsFunctionsTest COMMAND test_getSegments_functions)
