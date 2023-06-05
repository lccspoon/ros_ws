# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "traversability_msgs: 2 messages, 2 services")

set(MSG_I_FLAGS "-Itraversability_msgs:/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(traversability_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/msg/FootprintPath.msg" NAME_WE)
add_custom_target(_traversability_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "traversability_msgs" "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/msg/FootprintPath.msg" "std_msgs/Header:geometry_msgs/Pose:geometry_msgs/Quaternion:geometry_msgs/Polygon:geometry_msgs/PolygonStamped:geometry_msgs/Point32:geometry_msgs/PoseArray:geometry_msgs/Point"
)

get_filename_component(_filename "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/msg/TraversabilityResult.msg" NAME_WE)
add_custom_target(_traversability_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "traversability_msgs" "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/msg/TraversabilityResult.msg" ""
)

get_filename_component(_filename "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/srv/CheckFootprintPath.srv" NAME_WE)
add_custom_target(_traversability_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "traversability_msgs" "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/srv/CheckFootprintPath.srv" "std_msgs/Header:traversability_msgs/TraversabilityResult:geometry_msgs/Pose:geometry_msgs/Quaternion:geometry_msgs/Polygon:geometry_msgs/PolygonStamped:traversability_msgs/FootprintPath:geometry_msgs/Point32:geometry_msgs/PoseArray:geometry_msgs/Point"
)

get_filename_component(_filename "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/srv/Overwrite.srv" NAME_WE)
add_custom_target(_traversability_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "traversability_msgs" "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/srv/Overwrite.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(traversability_msgs
  "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/msg/FootprintPath.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Polygon.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PolygonStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseArray.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/traversability_msgs
)
_generate_msg_cpp(traversability_msgs
  "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/msg/TraversabilityResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/traversability_msgs
)

### Generating Services
_generate_srv_cpp(traversability_msgs
  "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/srv/CheckFootprintPath.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/msg/TraversabilityResult.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Polygon.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PolygonStamped.msg;/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/msg/FootprintPath.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseArray.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/traversability_msgs
)
_generate_srv_cpp(traversability_msgs
  "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/srv/Overwrite.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/traversability_msgs
)

### Generating Module File
_generate_module_cpp(traversability_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/traversability_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(traversability_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(traversability_msgs_generate_messages traversability_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/msg/FootprintPath.msg" NAME_WE)
add_dependencies(traversability_msgs_generate_messages_cpp _traversability_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/msg/TraversabilityResult.msg" NAME_WE)
add_dependencies(traversability_msgs_generate_messages_cpp _traversability_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/srv/CheckFootprintPath.srv" NAME_WE)
add_dependencies(traversability_msgs_generate_messages_cpp _traversability_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/srv/Overwrite.srv" NAME_WE)
add_dependencies(traversability_msgs_generate_messages_cpp _traversability_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(traversability_msgs_gencpp)
add_dependencies(traversability_msgs_gencpp traversability_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS traversability_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(traversability_msgs
  "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/msg/FootprintPath.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Polygon.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PolygonStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseArray.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/traversability_msgs
)
_generate_msg_eus(traversability_msgs
  "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/msg/TraversabilityResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/traversability_msgs
)

### Generating Services
_generate_srv_eus(traversability_msgs
  "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/srv/CheckFootprintPath.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/msg/TraversabilityResult.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Polygon.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PolygonStamped.msg;/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/msg/FootprintPath.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseArray.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/traversability_msgs
)
_generate_srv_eus(traversability_msgs
  "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/srv/Overwrite.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/traversability_msgs
)

### Generating Module File
_generate_module_eus(traversability_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/traversability_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(traversability_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(traversability_msgs_generate_messages traversability_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/msg/FootprintPath.msg" NAME_WE)
add_dependencies(traversability_msgs_generate_messages_eus _traversability_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/msg/TraversabilityResult.msg" NAME_WE)
add_dependencies(traversability_msgs_generate_messages_eus _traversability_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/srv/CheckFootprintPath.srv" NAME_WE)
add_dependencies(traversability_msgs_generate_messages_eus _traversability_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/srv/Overwrite.srv" NAME_WE)
add_dependencies(traversability_msgs_generate_messages_eus _traversability_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(traversability_msgs_geneus)
add_dependencies(traversability_msgs_geneus traversability_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS traversability_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(traversability_msgs
  "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/msg/FootprintPath.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Polygon.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PolygonStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseArray.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/traversability_msgs
)
_generate_msg_lisp(traversability_msgs
  "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/msg/TraversabilityResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/traversability_msgs
)

### Generating Services
_generate_srv_lisp(traversability_msgs
  "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/srv/CheckFootprintPath.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/msg/TraversabilityResult.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Polygon.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PolygonStamped.msg;/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/msg/FootprintPath.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseArray.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/traversability_msgs
)
_generate_srv_lisp(traversability_msgs
  "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/srv/Overwrite.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/traversability_msgs
)

### Generating Module File
_generate_module_lisp(traversability_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/traversability_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(traversability_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(traversability_msgs_generate_messages traversability_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/msg/FootprintPath.msg" NAME_WE)
add_dependencies(traversability_msgs_generate_messages_lisp _traversability_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/msg/TraversabilityResult.msg" NAME_WE)
add_dependencies(traversability_msgs_generate_messages_lisp _traversability_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/srv/CheckFootprintPath.srv" NAME_WE)
add_dependencies(traversability_msgs_generate_messages_lisp _traversability_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/srv/Overwrite.srv" NAME_WE)
add_dependencies(traversability_msgs_generate_messages_lisp _traversability_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(traversability_msgs_genlisp)
add_dependencies(traversability_msgs_genlisp traversability_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS traversability_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(traversability_msgs
  "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/msg/FootprintPath.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Polygon.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PolygonStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseArray.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/traversability_msgs
)
_generate_msg_nodejs(traversability_msgs
  "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/msg/TraversabilityResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/traversability_msgs
)

### Generating Services
_generate_srv_nodejs(traversability_msgs
  "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/srv/CheckFootprintPath.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/msg/TraversabilityResult.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Polygon.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PolygonStamped.msg;/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/msg/FootprintPath.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseArray.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/traversability_msgs
)
_generate_srv_nodejs(traversability_msgs
  "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/srv/Overwrite.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/traversability_msgs
)

### Generating Module File
_generate_module_nodejs(traversability_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/traversability_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(traversability_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(traversability_msgs_generate_messages traversability_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/msg/FootprintPath.msg" NAME_WE)
add_dependencies(traversability_msgs_generate_messages_nodejs _traversability_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/msg/TraversabilityResult.msg" NAME_WE)
add_dependencies(traversability_msgs_generate_messages_nodejs _traversability_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/srv/CheckFootprintPath.srv" NAME_WE)
add_dependencies(traversability_msgs_generate_messages_nodejs _traversability_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/srv/Overwrite.srv" NAME_WE)
add_dependencies(traversability_msgs_generate_messages_nodejs _traversability_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(traversability_msgs_gennodejs)
add_dependencies(traversability_msgs_gennodejs traversability_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS traversability_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(traversability_msgs
  "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/msg/FootprintPath.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Polygon.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PolygonStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseArray.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/traversability_msgs
)
_generate_msg_py(traversability_msgs
  "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/msg/TraversabilityResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/traversability_msgs
)

### Generating Services
_generate_srv_py(traversability_msgs
  "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/srv/CheckFootprintPath.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/msg/TraversabilityResult.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Polygon.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PolygonStamped.msg;/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/msg/FootprintPath.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseArray.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/traversability_msgs
)
_generate_srv_py(traversability_msgs
  "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/srv/Overwrite.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/traversability_msgs
)

### Generating Module File
_generate_module_py(traversability_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/traversability_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(traversability_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(traversability_msgs_generate_messages traversability_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/msg/FootprintPath.msg" NAME_WE)
add_dependencies(traversability_msgs_generate_messages_py _traversability_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/msg/TraversabilityResult.msg" NAME_WE)
add_dependencies(traversability_msgs_generate_messages_py _traversability_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/srv/CheckFootprintPath.srv" NAME_WE)
add_dependencies(traversability_msgs_generate_messages_py _traversability_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zyt/zyt_0526/src/traversability_estimation/traversability_msgs/srv/Overwrite.srv" NAME_WE)
add_dependencies(traversability_msgs_generate_messages_py _traversability_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(traversability_msgs_genpy)
add_dependencies(traversability_msgs_genpy traversability_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS traversability_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/traversability_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/traversability_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(traversability_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/traversability_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/traversability_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(traversability_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/traversability_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/traversability_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(traversability_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/traversability_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/traversability_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(traversability_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/traversability_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/traversability_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/traversability_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(traversability_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
