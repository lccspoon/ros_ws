# CMake generated Testfile for 
# Source directory: /home/zyt/zyt_0526/src/any_node/signal_handler
# Build directory: /home/zyt/zyt_0526/build/signal_handler
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_signal_handler_gtest_test_signal_handler "/home/zyt/zyt_0526/build/signal_handler/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/zyt/zyt_0526/build/signal_handler/test_results/signal_handler/gtest-test_signal_handler.xml" "--working-dir" "/home/zyt/zyt_0526/src/any_node/signal_handler/test" "--return-code" "/home/zyt/zyt_0526/devel/.private/signal_handler/lib/signal_handler/test_signal_handler --gtest_output=xml:/home/zyt/zyt_0526/build/signal_handler/test_results/signal_handler/gtest-test_signal_handler.xml")
set_tests_properties(_ctest_signal_handler_gtest_test_signal_handler PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;98;catkin_run_tests_target;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;37;_catkin_add_google_test;/home/zyt/zyt_0526/src/any_node/signal_handler/CMakeLists.txt;44;catkin_add_gtest;/home/zyt/zyt_0526/src/any_node/signal_handler/CMakeLists.txt;0;")
subdirs("gtest")
