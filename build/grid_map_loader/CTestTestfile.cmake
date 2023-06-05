# CMake generated Testfile for 
# Source directory: /home/zyt/zyt_0526/src/grid_map/grid_map_loader
# Build directory: /home/zyt/zyt_0526/build/grid_map_loader
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_grid_map_loader_gtest_grid_map_loader-test "/home/zyt/zyt_0526/build/grid_map_loader/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/zyt/zyt_0526/build/grid_map_loader/test_results/grid_map_loader/gtest-grid_map_loader-test.xml" "--return-code" "/home/zyt/zyt_0526/devel/.private/grid_map_loader/lib/grid_map_loader/grid_map_loader-test --gtest_output=xml:/home/zyt/zyt_0526/build/grid_map_loader/test_results/grid_map_loader/gtest-grid_map_loader-test.xml")
set_tests_properties(_ctest_grid_map_loader_gtest_grid_map_loader-test PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;98;catkin_run_tests_target;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;37;_catkin_add_google_test;/home/zyt/zyt_0526/src/grid_map/grid_map_loader/CMakeLists.txt;76;catkin_add_gtest;/home/zyt/zyt_0526/src/grid_map/grid_map_loader/CMakeLists.txt;0;")
subdirs("gtest")
