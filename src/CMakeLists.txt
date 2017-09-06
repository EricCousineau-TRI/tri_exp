cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(tri_exp)

# Requires that environment variables be appropriately set
# (e.g. $ENV{PATH} for use with find_package.)
find_package(PCL 1.8 REQUIRED)
find_package(drake REQUIRED)

add_definitions(-std=c++11 -g -fext-numeric-literals ${PCL_DEFINITIONS})
include_directories(
    ${PCL_INCLUDE_DIRS}
)
link_directories(
	$ENV{GUROBI_HOME}/lib
)

add_library(lib_perception perception.cc point_cloud_registration.cc)
target_link_libraries(lib_perception ${PCL_LIBRARIES} drake::drake drake::drake-lcmtypes-cpp)

add_executable(link_test link_test.cc)
target_link_libraries(link_test lib_perception)

# Debug info
get_target_property(DBG_drake_INC_DIR drake::drake INTERFACE_INCLUDE_DIRECTORIES)
message(STATUS "DRAKE: ${drake_DIR}\n${DBG_drake_INC_DIR}")
message(STATUS "LCM: ${lcm_DIR}")
message(STATUS "PCL_INCLUDE_DIR: " ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIB_DIR: " ${PCL_LIBRARY_DIRS})
