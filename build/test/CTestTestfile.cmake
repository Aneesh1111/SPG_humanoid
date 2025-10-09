# CMake generated Testfile for 
# Source directory: /home/robocup/Downloads/spg_cmake/test
# Build directory: /home/robocup/Downloads/spg_cmake/build/test
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(BalanceXYTest "/home/robocup/Downloads/spg_cmake/build/test/test_balanceXY")
set_tests_properties(BalanceXYTest PROPERTIES  _BACKTRACE_TRIPLES "/home/robocup/Downloads/spg_cmake/test/CMakeLists.txt;61;add_test;/home/robocup/Downloads/spg_cmake/test/CMakeLists.txt;0;")
add_test(GetSegmentsTest "/home/robocup/Downloads/spg_cmake/build/test/test_getSegments")
set_tests_properties(GetSegmentsTest PROPERTIES  _BACKTRACE_TRIPLES "/home/robocup/Downloads/spg_cmake/test/CMakeLists.txt;62;add_test;/home/robocup/Downloads/spg_cmake/test/CMakeLists.txt;0;")
add_test(GetSegmentsFunctionsTest "/home/robocup/Downloads/spg_cmake/build/test/test_getSegments_functions")
set_tests_properties(GetSegmentsFunctionsTest PROPERTIES  _BACKTRACE_TRIPLES "/home/robocup/Downloads/spg_cmake/test/CMakeLists.txt;63;add_test;/home/robocup/Downloads/spg_cmake/test/CMakeLists.txt;0;")
add_test(Traj1Test "/home/robocup/Downloads/spg_cmake/build/test/test_traj1")
set_tests_properties(Traj1Test PROPERTIES  _BACKTRACE_TRIPLES "/home/robocup/Downloads/spg_cmake/test/CMakeLists.txt;64;add_test;/home/robocup/Downloads/spg_cmake/test/CMakeLists.txt;0;")
add_test(TrajPredictTest "/home/robocup/Downloads/spg_cmake/build/test/test_trajpredict")
set_tests_properties(TrajPredictTest PROPERTIES  _BACKTRACE_TRIPLES "/home/robocup/Downloads/spg_cmake/test/CMakeLists.txt;65;add_test;/home/robocup/Downloads/spg_cmake/test/CMakeLists.txt;0;")
add_test(StaticObstaclesIntegrationTest "/home/robocup/Downloads/spg_cmake/build/test/test_static_obstacles_integration")
set_tests_properties(StaticObstaclesIntegrationTest PROPERTIES  _BACKTRACE_TRIPLES "/home/robocup/Downloads/spg_cmake/test/CMakeLists.txt;66;add_test;/home/robocup/Downloads/spg_cmake/test/CMakeLists.txt;0;")
add_test(MoveStraightLineTest "/home/robocup/Downloads/spg_cmake/build/test/test_move_straight_line")
set_tests_properties(MoveStraightLineTest PROPERTIES  _BACKTRACE_TRIPLES "/home/robocup/Downloads/spg_cmake/test/CMakeLists.txt;67;add_test;/home/robocup/Downloads/spg_cmake/test/CMakeLists.txt;0;")
add_test(SetpointSetTest "/home/robocup/Downloads/spg_cmake/build/test/test_setpoint_set")
set_tests_properties(SetpointSetTest PROPERTIES  _BACKTRACE_TRIPLES "/home/robocup/Downloads/spg_cmake/test/CMakeLists.txt;68;add_test;/home/robocup/Downloads/spg_cmake/test/CMakeLists.txt;0;")
add_test(DetermineSetpointLimitsTest "/home/robocup/Downloads/spg_cmake/build/test/test_determine_setpoint_limits")
set_tests_properties(DetermineSetpointLimitsTest PROPERTIES  _BACKTRACE_TRIPLES "/home/robocup/Downloads/spg_cmake/test/CMakeLists.txt;69;add_test;/home/robocup/Downloads/spg_cmake/test/CMakeLists.txt;0;")
subdirs("../_deps/googletest-build")
