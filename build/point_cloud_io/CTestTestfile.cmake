# CMake generated Testfile for 
# Source directory: /home/zyt/zyt_0526/src/point_cloud_io
# Build directory: /home/zyt/zyt_0526/build/point_cloud_io
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_point_cloud_io_gtest_test_point_cloud_io "/home/zyt/zyt_0526/build/point_cloud_io/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/zyt/zyt_0526/build/point_cloud_io/test_results/point_cloud_io/gtest-test_point_cloud_io.xml" "--return-code" "/home/zyt/zyt_0526/devel/.private/point_cloud_io/lib/point_cloud_io/test_point_cloud_io --gtest_output=xml:/home/zyt/zyt_0526/build/point_cloud_io/test_results/point_cloud_io/gtest-test_point_cloud_io.xml")
set_tests_properties(_ctest_point_cloud_io_gtest_test_point_cloud_io PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;98;catkin_run_tests_target;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;37;_catkin_add_google_test;/home/zyt/zyt_0526/src/point_cloud_io/CMakeLists.txt;94;catkin_add_gtest;/home/zyt/zyt_0526/src/point_cloud_io/CMakeLists.txt;0;")
subdirs("gtest")
