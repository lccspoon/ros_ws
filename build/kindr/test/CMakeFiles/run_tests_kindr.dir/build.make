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
CMAKE_SOURCE_DIR = /home/zyt/zyt_0526/src/kindr

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zyt/zyt_0526/build/kindr

# Utility rule file for run_tests_kindr.

# Include the progress variables for this target.
include test/CMakeFiles/run_tests_kindr.dir/progress.make

run_tests_kindr: test/CMakeFiles/run_tests_kindr.dir/build.make

.PHONY : run_tests_kindr

# Rule to build all files generated by this target.
test/CMakeFiles/run_tests_kindr.dir/build: run_tests_kindr

.PHONY : test/CMakeFiles/run_tests_kindr.dir/build

test/CMakeFiles/run_tests_kindr.dir/clean:
	cd /home/zyt/zyt_0526/build/kindr/test && $(CMAKE_COMMAND) -P CMakeFiles/run_tests_kindr.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/run_tests_kindr.dir/clean

test/CMakeFiles/run_tests_kindr.dir/depend:
	cd /home/zyt/zyt_0526/build/kindr && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zyt/zyt_0526/src/kindr /home/zyt/zyt_0526/src/kindr/test /home/zyt/zyt_0526/build/kindr /home/zyt/zyt_0526/build/kindr/test /home/zyt/zyt_0526/build/kindr/test/CMakeFiles/run_tests_kindr.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/run_tests_kindr.dir/depend

