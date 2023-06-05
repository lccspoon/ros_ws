# Install script for directory: /home/zyt/zyt_0526/src/grid_map/grid_map_demos

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/zyt/zyt_0526/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/zyt/zyt_0526/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/zyt/zyt_0526/install" TYPE PROGRAM FILES "/home/zyt/zyt_0526/build/grid_map_demos/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/zyt/zyt_0526/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/zyt/zyt_0526/install" TYPE PROGRAM FILES "/home/zyt/zyt_0526/build/grid_map_demos/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/zyt/zyt_0526/install/setup.bash;/home/zyt/zyt_0526/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/zyt/zyt_0526/install" TYPE FILE FILES
    "/home/zyt/zyt_0526/build/grid_map_demos/catkin_generated/installspace/setup.bash"
    "/home/zyt/zyt_0526/build/grid_map_demos/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/zyt/zyt_0526/install/setup.sh;/home/zyt/zyt_0526/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/zyt/zyt_0526/install" TYPE FILE FILES
    "/home/zyt/zyt_0526/build/grid_map_demos/catkin_generated/installspace/setup.sh"
    "/home/zyt/zyt_0526/build/grid_map_demos/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/zyt/zyt_0526/install/setup.zsh;/home/zyt/zyt_0526/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/zyt/zyt_0526/install" TYPE FILE FILES
    "/home/zyt/zyt_0526/build/grid_map_demos/catkin_generated/installspace/setup.zsh"
    "/home/zyt/zyt_0526/build/grid_map_demos/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/zyt/zyt_0526/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/zyt/zyt_0526/install" TYPE FILE FILES "/home/zyt/zyt_0526/build/grid_map_demos/catkin_generated/installspace/.rosinstall")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/zyt/zyt_0526/build/grid_map_demos/catkin_generated/installspace/grid_map_demos.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/grid_map_demos/cmake" TYPE FILE FILES
    "/home/zyt/zyt_0526/build/grid_map_demos/catkin_generated/installspace/grid_map_demosConfig.cmake"
    "/home/zyt/zyt_0526/build/grid_map_demos/catkin_generated/installspace/grid_map_demosConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/grid_map_demos" TYPE FILE FILES "/home/zyt/zyt_0526/src/grid_map/grid_map_demos/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos" TYPE PROGRAM FILES "/home/zyt/zyt_0526/build/grid_map_demos/catkin_generated/installspace/image_publisher.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/filters_demo" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/filters_demo")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/filters_demo"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos" TYPE EXECUTABLE FILES "/home/zyt/zyt_0526/devel/.private/grid_map_demos/lib/grid_map_demos/filters_demo")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/filters_demo" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/filters_demo")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/filters_demo"
         OLD_RPATH "/home/zyt/zyt_0526/devel/.private/grid_map_filters/lib:/home/zyt/zyt_0526/devel/.private/grid_map_octomap/lib:/home/zyt/zyt_0526/devel/.private/grid_map_rviz_plugin/lib:/home/zyt/zyt_0526/devel/.private/grid_map_ros/lib:/home/zyt/zyt_0526/devel/.private/grid_map_cv/lib:/home/zyt/zyt_0526/devel/.private/grid_map_core/lib:/opt/ros/noetic/lib:/home/zyt/zyt_0526/devel/.private/grid_map_sdf/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/filters_demo")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/image_to_gridmap_demo" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/image_to_gridmap_demo")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/image_to_gridmap_demo"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos" TYPE EXECUTABLE FILES "/home/zyt/zyt_0526/devel/.private/grid_map_demos/lib/grid_map_demos/image_to_gridmap_demo")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/image_to_gridmap_demo" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/image_to_gridmap_demo")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/image_to_gridmap_demo"
         OLD_RPATH "/home/zyt/zyt_0526/devel/.private/grid_map_filters/lib:/home/zyt/zyt_0526/devel/.private/grid_map_octomap/lib:/home/zyt/zyt_0526/devel/.private/grid_map_rviz_plugin/lib:/home/zyt/zyt_0526/devel/.private/grid_map_ros/lib:/home/zyt/zyt_0526/devel/.private/grid_map_cv/lib:/home/zyt/zyt_0526/devel/.private/grid_map_core/lib:/opt/ros/noetic/lib:/home/zyt/zyt_0526/devel/.private/grid_map_sdf/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/image_to_gridmap_demo")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/grid_map_to_image_demo" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/grid_map_to_image_demo")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/grid_map_to_image_demo"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos" TYPE EXECUTABLE FILES "/home/zyt/zyt_0526/devel/.private/grid_map_demos/lib/grid_map_demos/grid_map_to_image_demo")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/grid_map_to_image_demo" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/grid_map_to_image_demo")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/grid_map_to_image_demo"
         OLD_RPATH "/home/zyt/zyt_0526/devel/.private/grid_map_filters/lib:/home/zyt/zyt_0526/devel/.private/grid_map_octomap/lib:/home/zyt/zyt_0526/devel/.private/grid_map_rviz_plugin/lib:/home/zyt/zyt_0526/devel/.private/grid_map_ros/lib:/home/zyt/zyt_0526/devel/.private/grid_map_cv/lib:/home/zyt/zyt_0526/devel/.private/grid_map_core/lib:/opt/ros/noetic/lib:/home/zyt/zyt_0526/devel/.private/grid_map_sdf/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/grid_map_to_image_demo")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/interpolation_demo" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/interpolation_demo")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/interpolation_demo"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos" TYPE EXECUTABLE FILES "/home/zyt/zyt_0526/devel/.private/grid_map_demos/lib/grid_map_demos/interpolation_demo")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/interpolation_demo" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/interpolation_demo")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/interpolation_demo"
         OLD_RPATH "/home/zyt/zyt_0526/devel/.private/grid_map_filters/lib:/home/zyt/zyt_0526/devel/.private/grid_map_octomap/lib:/home/zyt/zyt_0526/devel/.private/grid_map_rviz_plugin/lib:/home/zyt/zyt_0526/devel/.private/grid_map_ros/lib:/home/zyt/zyt_0526/devel/.private/grid_map_cv/lib:/home/zyt/zyt_0526/devel/.private/grid_map_core/lib:/opt/ros/noetic/lib:/home/zyt/zyt_0526/devel/.private/grid_map_sdf/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/interpolation_demo")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/iterator_benchmark" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/iterator_benchmark")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/iterator_benchmark"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos" TYPE EXECUTABLE FILES "/home/zyt/zyt_0526/devel/.private/grid_map_demos/lib/grid_map_demos/iterator_benchmark")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/iterator_benchmark" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/iterator_benchmark")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/iterator_benchmark"
         OLD_RPATH "/home/zyt/zyt_0526/devel/.private/grid_map_filters/lib:/home/zyt/zyt_0526/devel/.private/grid_map_octomap/lib:/home/zyt/zyt_0526/devel/.private/grid_map_rviz_plugin/lib:/home/zyt/zyt_0526/devel/.private/grid_map_ros/lib:/home/zyt/zyt_0526/devel/.private/grid_map_cv/lib:/home/zyt/zyt_0526/devel/.private/grid_map_core/lib:/opt/ros/noetic/lib:/home/zyt/zyt_0526/devel/.private/grid_map_sdf/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/iterator_benchmark")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/iterators_demo" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/iterators_demo")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/iterators_demo"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos" TYPE EXECUTABLE FILES "/home/zyt/zyt_0526/devel/.private/grid_map_demos/lib/grid_map_demos/iterators_demo")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/iterators_demo" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/iterators_demo")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/iterators_demo"
         OLD_RPATH "/home/zyt/zyt_0526/devel/.private/grid_map_filters/lib:/home/zyt/zyt_0526/devel/.private/grid_map_octomap/lib:/home/zyt/zyt_0526/devel/.private/grid_map_rviz_plugin/lib:/home/zyt/zyt_0526/devel/.private/grid_map_ros/lib:/home/zyt/zyt_0526/devel/.private/grid_map_cv/lib:/home/zyt/zyt_0526/devel/.private/grid_map_core/lib:/opt/ros/noetic/lib:/home/zyt/zyt_0526/devel/.private/grid_map_sdf/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/iterators_demo")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/move_demo" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/move_demo")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/move_demo"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos" TYPE EXECUTABLE FILES "/home/zyt/zyt_0526/devel/.private/grid_map_demos/lib/grid_map_demos/move_demo")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/move_demo" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/move_demo")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/move_demo"
         OLD_RPATH "/home/zyt/zyt_0526/devel/.private/grid_map_filters/lib:/home/zyt/zyt_0526/devel/.private/grid_map_octomap/lib:/home/zyt/zyt_0526/devel/.private/grid_map_rviz_plugin/lib:/home/zyt/zyt_0526/devel/.private/grid_map_ros/lib:/home/zyt/zyt_0526/devel/.private/grid_map_cv/lib:/home/zyt/zyt_0526/devel/.private/grid_map_core/lib:/opt/ros/noetic/lib:/home/zyt/zyt_0526/devel/.private/grid_map_sdf/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/move_demo")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/normal_filter_comparison_demo" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/normal_filter_comparison_demo")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/normal_filter_comparison_demo"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos" TYPE EXECUTABLE FILES "/home/zyt/zyt_0526/devel/.private/grid_map_demos/lib/grid_map_demos/normal_filter_comparison_demo")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/normal_filter_comparison_demo" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/normal_filter_comparison_demo")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/normal_filter_comparison_demo"
         OLD_RPATH "/home/zyt/zyt_0526/devel/.private/grid_map_filters/lib:/home/zyt/zyt_0526/devel/.private/grid_map_octomap/lib:/home/zyt/zyt_0526/devel/.private/grid_map_rviz_plugin/lib:/home/zyt/zyt_0526/devel/.private/grid_map_ros/lib:/home/zyt/zyt_0526/devel/.private/grid_map_cv/lib:/home/zyt/zyt_0526/devel/.private/grid_map_core/lib:/opt/ros/noetic/lib:/home/zyt/zyt_0526/devel/.private/grid_map_sdf/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/normal_filter_comparison_demo")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/octomap_to_gridmap_demo" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/octomap_to_gridmap_demo")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/octomap_to_gridmap_demo"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos" TYPE EXECUTABLE FILES "/home/zyt/zyt_0526/devel/.private/grid_map_demos/lib/grid_map_demos/octomap_to_gridmap_demo")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/octomap_to_gridmap_demo" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/octomap_to_gridmap_demo")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/octomap_to_gridmap_demo"
         OLD_RPATH "/home/zyt/zyt_0526/devel/.private/grid_map_filters/lib:/home/zyt/zyt_0526/devel/.private/grid_map_octomap/lib:/home/zyt/zyt_0526/devel/.private/grid_map_rviz_plugin/lib:/home/zyt/zyt_0526/devel/.private/grid_map_ros/lib:/home/zyt/zyt_0526/devel/.private/grid_map_cv/lib:/home/zyt/zyt_0526/devel/.private/grid_map_core/lib:/opt/ros/noetic/lib:/home/zyt/zyt_0526/devel/.private/grid_map_sdf/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/octomap_to_gridmap_demo")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/opencv_demo" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/opencv_demo")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/opencv_demo"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos" TYPE EXECUTABLE FILES "/home/zyt/zyt_0526/devel/.private/grid_map_demos/lib/grid_map_demos/opencv_demo")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/opencv_demo" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/opencv_demo")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/opencv_demo"
         OLD_RPATH "/home/zyt/zyt_0526/devel/.private/grid_map_filters/lib:/home/zyt/zyt_0526/devel/.private/grid_map_octomap/lib:/home/zyt/zyt_0526/devel/.private/grid_map_rviz_plugin/lib:/home/zyt/zyt_0526/devel/.private/grid_map_ros/lib:/home/zyt/zyt_0526/devel/.private/grid_map_cv/lib:/home/zyt/zyt_0526/devel/.private/grid_map_core/lib:/opt/ros/noetic/lib:/home/zyt/zyt_0526/devel/.private/grid_map_sdf/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/opencv_demo")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/resolution_change_demo" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/resolution_change_demo")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/resolution_change_demo"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos" TYPE EXECUTABLE FILES "/home/zyt/zyt_0526/devel/.private/grid_map_demos/lib/grid_map_demos/resolution_change_demo")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/resolution_change_demo" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/resolution_change_demo")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/resolution_change_demo"
         OLD_RPATH "/home/zyt/zyt_0526/devel/.private/grid_map_filters/lib:/home/zyt/zyt_0526/devel/.private/grid_map_octomap/lib:/home/zyt/zyt_0526/devel/.private/grid_map_rviz_plugin/lib:/home/zyt/zyt_0526/devel/.private/grid_map_ros/lib:/home/zyt/zyt_0526/devel/.private/grid_map_cv/lib:/home/zyt/zyt_0526/devel/.private/grid_map_core/lib:/opt/ros/noetic/lib:/home/zyt/zyt_0526/devel/.private/grid_map_sdf/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/resolution_change_demo")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/simple_demo" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/simple_demo")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/simple_demo"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos" TYPE EXECUTABLE FILES "/home/zyt/zyt_0526/devel/.private/grid_map_demos/lib/grid_map_demos/simple_demo")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/simple_demo" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/simple_demo")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/simple_demo"
         OLD_RPATH "/home/zyt/zyt_0526/devel/.private/grid_map_filters/lib:/home/zyt/zyt_0526/devel/.private/grid_map_octomap/lib:/home/zyt/zyt_0526/devel/.private/grid_map_rviz_plugin/lib:/home/zyt/zyt_0526/devel/.private/grid_map_ros/lib:/home/zyt/zyt_0526/devel/.private/grid_map_cv/lib:/home/zyt/zyt_0526/devel/.private/grid_map_core/lib:/opt/ros/noetic/lib:/home/zyt/zyt_0526/devel/.private/grid_map_sdf/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/simple_demo")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/tutorial_demo" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/tutorial_demo")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/tutorial_demo"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos" TYPE EXECUTABLE FILES "/home/zyt/zyt_0526/devel/.private/grid_map_demos/lib/grid_map_demos/tutorial_demo")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/tutorial_demo" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/tutorial_demo")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/tutorial_demo"
         OLD_RPATH "/home/zyt/zyt_0526/devel/.private/grid_map_filters/lib:/home/zyt/zyt_0526/devel/.private/grid_map_octomap/lib:/home/zyt/zyt_0526/devel/.private/grid_map_rviz_plugin/lib:/home/zyt/zyt_0526/devel/.private/grid_map_ros/lib:/home/zyt/zyt_0526/devel/.private/grid_map_cv/lib:/home/zyt/zyt_0526/devel/.private/grid_map_core/lib:/opt/ros/noetic/lib:/home/zyt/zyt_0526/devel/.private/grid_map_sdf/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/tutorial_demo")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/sdf_demo" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/sdf_demo")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/sdf_demo"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos" TYPE EXECUTABLE FILES "/home/zyt/zyt_0526/devel/.private/grid_map_demos/lib/grid_map_demos/sdf_demo")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/sdf_demo" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/sdf_demo")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/sdf_demo"
         OLD_RPATH "/home/zyt/zyt_0526/devel/.private/grid_map_filters/lib:/home/zyt/zyt_0526/devel/.private/grid_map_octomap/lib:/home/zyt/zyt_0526/devel/.private/grid_map_rviz_plugin/lib:/home/zyt/zyt_0526/devel/.private/grid_map_ros/lib:/home/zyt/zyt_0526/devel/.private/grid_map_cv/lib:/home/zyt/zyt_0526/devel/.private/grid_map_core/lib:/opt/ros/noetic/lib:/home/zyt/zyt_0526/devel/.private/grid_map_sdf/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/grid_map_demos/sdf_demo")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/grid_map_demos" TYPE DIRECTORY FILES
    "/home/zyt/zyt_0526/src/grid_map/grid_map_demos/config"
    "/home/zyt/zyt_0526/src/grid_map/grid_map_demos/data"
    "/home/zyt/zyt_0526/src/grid_map/grid_map_demos/doc"
    "/home/zyt/zyt_0526/src/grid_map/grid_map_demos/launch"
    "/home/zyt/zyt_0526/src/grid_map/grid_map_demos/rviz"
    "/home/zyt/zyt_0526/src/grid_map/grid_map_demos/scripts"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/zyt/zyt_0526/build/grid_map_demos/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/zyt/zyt_0526/build/grid_map_demos/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
