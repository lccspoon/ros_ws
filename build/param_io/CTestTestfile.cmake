# CMake generated Testfile for 
# Source directory: /home/zyt/zyt_0526/src/any_node/param_io
# Build directory: /home/zyt/zyt_0526/build/param_io
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_param_io_rostest_test_param_io.test "/home/zyt/zyt_0526/build/param_io/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/zyt/zyt_0526/build/param_io/test_results/param_io/rostest-test_param_io.xml" "--return-code" "/usr/bin/python3 /opt/ros/noetic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/zyt/zyt_0526/src/any_node/param_io --package=param_io --results-filename test_param_io.xml --results-base-dir \"/home/zyt/zyt_0526/build/param_io/test_results\" /home/zyt/zyt_0526/src/any_node/param_io/test/param_io.test ")
set_tests_properties(_ctest_param_io_rostest_test_param_io.test PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/rostest/cmake/rostest-extras.cmake;52;catkin_run_tests_target;/opt/ros/noetic/share/rostest/cmake/rostest-extras.cmake;80;add_rostest;/opt/ros/noetic/share/rostest/cmake/rostest-extras.cmake;100;_add_rostest_google_test;/home/zyt/zyt_0526/src/any_node/param_io/CMakeLists.txt;31;add_rostest_gtest;/home/zyt/zyt_0526/src/any_node/param_io/CMakeLists.txt;0;")
subdirs("gtest")
