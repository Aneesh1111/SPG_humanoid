# CMake generated Testfile for 
# Source directory: /home/robocup/Downloads/spg_cmake/test
# Build directory: /home/robocup/Downloads/spg_cmake/build/test
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(BalanceXYTest "/home/robocup/Downloads/spg_cmake/build/test/test_balanceXY")
set_tests_properties(BalanceXYTest PROPERTIES  _BACKTRACE_TRIPLES "/home/robocup/Downloads/spg_cmake/test/CMakeLists.txt;31;add_test;/home/robocup/Downloads/spg_cmake/test/CMakeLists.txt;0;")
add_test(GetSegmentsTest "/home/robocup/Downloads/spg_cmake/build/test/test_getSegments")
set_tests_properties(GetSegmentsTest PROPERTIES  _BACKTRACE_TRIPLES "/home/robocup/Downloads/spg_cmake/test/CMakeLists.txt;32;add_test;/home/robocup/Downloads/spg_cmake/test/CMakeLists.txt;0;")
add_test(GetSegmentsFunctionsTest "/home/robocup/Downloads/spg_cmake/build/test/test_getSegments_functions")
set_tests_properties(GetSegmentsFunctionsTest PROPERTIES  _BACKTRACE_TRIPLES "/home/robocup/Downloads/spg_cmake/test/CMakeLists.txt;33;add_test;/home/robocup/Downloads/spg_cmake/test/CMakeLists.txt;0;")
subdirs("../_deps/googletest-build")
