# CMake generated Testfile for 
# Source directory: /home/zyt/zyt_0526/src/grid_map/grid_map_demos
# Build directory: /home/zyt/zyt_0526/build/grid_map_demos
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_grid_map_demos_gtest_grid_map_demos-test "/home/zyt/zyt_0526/build/grid_map_demos/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/zyt/zyt_0526/build/grid_map_demos/test_results/grid_map_demos/gtest-grid_map_demos-test.xml" "--return-code" "/home/zyt/zyt_0526/devel/.private/grid_map_demos/lib/grid_map_demos/grid_map_demos-test --gtest_output=xml:/home/zyt/zyt_0526/build/grid_map_demos/test_results/grid_map_demos/gtest-grid_map_demos-test.xml")
set_tests_properties(_ctest_grid_map_demos_gtest_grid_map_demos-test PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;98;catkin_run_tests_target;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;37;_catkin_add_google_test;/home/zyt/zyt_0526/src/grid_map/grid_map_demos/CMakeLists.txt;243;catkin_add_gtest;/home/zyt/zyt_0526/src/grid_map/grid_map_demos/CMakeLists.txt;0;")
subdirs("gtest")
