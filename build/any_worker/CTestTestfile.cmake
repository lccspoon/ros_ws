# CMake generated Testfile for 
# Source directory: /home/zyt/zyt_0526/src/any_node/any_worker
# Build directory: /home/zyt/zyt_0526/build/any_worker
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_any_worker_gtest_test_any_worker "/home/zyt/zyt_0526/build/any_worker/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/zyt/zyt_0526/build/any_worker/test_results/any_worker/gtest-test_any_worker.xml" "--return-code" "/home/zyt/zyt_0526/devel/.private/any_worker/lib/any_worker/test_any_worker --gtest_output=xml:/home/zyt/zyt_0526/build/any_worker/test_results/any_worker/gtest-test_any_worker.xml")
set_tests_properties(_ctest_any_worker_gtest_test_any_worker PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;98;catkin_run_tests_target;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;37;_catkin_add_google_test;/home/zyt/zyt_0526/src/any_node/any_worker/CMakeLists.txt;43;catkin_add_gtest;/home/zyt/zyt_0526/src/any_node/any_worker/CMakeLists.txt;0;")
subdirs("gtest")
