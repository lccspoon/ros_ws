# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "grid_map_ros;grid_map_core;grid_map_msgs;grid_map_filters;roscpp;tf;tf_conversions;traversability_estimation_filters;traversability_msgs;std_msgs;geometry_msgs;sensor_msgs;param_io;xmlrpcpp".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-ltraversability_estimation".split(';') if "-ltraversability_estimation" != "" else []
PROJECT_NAME = "traversability_estimation"
PROJECT_SPACE_DIR = "/home/zyt/zyt_0526/install"
PROJECT_VERSION = "0.4.0"
