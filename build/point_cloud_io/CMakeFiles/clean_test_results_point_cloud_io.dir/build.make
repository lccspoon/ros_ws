# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/zyt/zyt_0526/src/point_cloud_io

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zyt/zyt_0526/build/point_cloud_io

# Utility rule file for clean_test_results_point_cloud_io.

# Include the progress variables for this target.
include CMakeFiles/clean_test_results_point_cloud_io.dir/progress.make

CMakeFiles/clean_test_results_point_cloud_io:
	/usr/bin/python3 /opt/ros/noetic/share/catkin/cmake/test/remove_test_results.py /home/zyt/zyt_0526/build/point_cloud_io/test_results/point_cloud_io

clean_test_results_point_cloud_io: CMakeFiles/clean_test_results_point_cloud_io
clean_test_results_point_cloud_io: CMakeFiles/clean_test_results_point_cloud_io.dir/build.make

.PHONY : clean_test_results_point_cloud_io

# Rule to build all files generated by this target.
CMakeFiles/clean_test_results_point_cloud_io.dir/build: clean_test_results_point_cloud_io

.PHONY : CMakeFiles/clean_test_results_point_cloud_io.dir/build

CMakeFiles/clean_test_results_point_cloud_io.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/clean_test_results_point_cloud_io.dir/cmake_clean.cmake
.PHONY : CMakeFiles/clean_test_results_point_cloud_io.dir/clean

CMakeFiles/clean_test_results_point_cloud_io.dir/depend:
	cd /home/zyt/zyt_0526/build/point_cloud_io && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zyt/zyt_0526/src/point_cloud_io /home/zyt/zyt_0526/src/point_cloud_io /home/zyt/zyt_0526/build/point_cloud_io /home/zyt/zyt_0526/build/point_cloud_io /home/zyt/zyt_0526/build/point_cloud_io/CMakeFiles/clean_test_results_point_cloud_io.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/clean_test_results_point_cloud_io.dir/depend

