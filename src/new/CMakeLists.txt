# cmake_minimum_required(VERSION 2.8.3)

# project(new)

# find_package(catkin REQUIRED)

# catkin_package()

# find_package(roslaunch)

# foreach(dir config launch meshes urdf)
# 	install(DIRECTORY ${dir}/
# 		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
# endforeach(dir)



# 新的配置
cmake_minimum_required(VERSION 2.8.3)

project(new)



find_package(roslaunch)

find_package(catkin REQUIRED COMPONENTS 
	angles
	control_msgs
	control_toolbox
	controller_interface
	eigen_conversions
	forward_command_controller
	hardware_interface
	pluginlib
	roscpp
    rospy
	realtime_tools
	std_msgs
	sensor_msgs
	message_generation
	urdf
	rviz
	joint_state_publisher 
	robot_state_publisher
)


find_package(Boost REQUIRED COMPONENTS system)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)

# add_message_files(
# 	FILES
# 	test.msg
# )

generate_messages(
  DEPENDENCIES
  std_msgs
)




catkin_package(
	# INCLUDE_DIRS
	# include
	${Eigen_INCLUDE_DIRS}
  LIBRARIES
	${PROJECT_NAME}_library
  CATKIN_DEPENDS
    control_msgs
    control_toolbox
    controller_interface
    forward_command_controller
    geometry_msgs
    hardware_interface
    realtime_tools
    std_msgs
    sensor_msgs
    urdf
    roscpp
    rospy
    rosmsg
    message_runtime
	rviz
	joint_state_publisher 
	robot_state_publisher
#  DEPENDS system_lib
)
include_directories(
	include
 ${catkin_INCLUDE_DIRS} 
#  ${PCL_INCLUDE_DIRS}
${Eigen_INCLUDE_DIRS}
)
add_definitions(${EIGEN_DEFINITIONS})


# add_executable(adaptive_control 
# src/robot_adaptive_control.cpp
# # src/kinematic.cpp
# src/correctkinematic.cpp
# src/Toque
# src/inverse_kinematic.cpp
# src/force.cpp
# src/database.cpp
# src/kinematic2.cpp
# src/CPG_new.cpp
# src/FootTipTrajectory.cpp

# )


# target_link_libraries(adaptive_control ${catkin_LIBRARIES})


# add_dependencies(adaptive_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)



foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)















