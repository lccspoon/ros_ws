aux_source_directory(. DIR_LIB_SRCS_CONTROLLER)
#c
# FILE(GLOB_RECURSE headers_con *.hpp)
# FILE(GLOB_RECURSE source_con *.c)
# FILE(GLOB_RECURSE sources_con *.cpp)
# include_directories("../xstypes")
# include_directories("../xscontroller")
# include_directories("../xscommon")
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build/lib)
include_directories("../include")
# add_library(XsController ${header_con} ${headers_con} ${source_con} ${sources_con})
add_library(XsController ${DIR_LIB_SRCS_CONTROLLER})