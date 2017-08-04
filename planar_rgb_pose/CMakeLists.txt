cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(openni_grabber)

set(drake_DIR "/home/user/drake_bazel_build/lib/cmake/drake")
#list(APPEND CMAKE_MODULE_PATH "/home/user/opencv/build/")
set(CMAKE_MODULE_PATH "/home/user/opencv/build/;${CMAKE_MODULE_PATH}")

MESSAGE(STATUS "CMAKE MODULE PATH" ${CMAKE_MODULE_PATH})
#find_package(PCL 1.8 REQUIRED)
find_package(OpenCV 3.3.0 EXACT REQUIRED PATHS /home/user/opencv/build)
find_package(drake REQUIRED)

include_directories(
  /usr/local/include/pcl-1.8
  /usr/include/ni
  /usr/include/openni2
/home/user/drake_bazel_build/include/eigen3
#${PCL_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
${apriltags_INCLUDE_DIR}
)

add_definitions(${PCL_DEFINITIONS} -std=c++11 -g -O2 -fext-numeric-literals)

MESSAGE(STATUS "OpenCV_INCLUDE_DIR" ${OpenCV_INCLUDE_DIRS})

add_executable (test_planar_tracking test_planar_rgb_pose_estimation.cc)
target_link_libraries (test_planar_tracking ${OpenCV_LIBS})

#add_executable (test_feature_gen planar_rgb_model.cc)
#target_link_libraries (test_feature_gen ${OpenCV_LIBS})

add_executable (test_matching planar_rgb_pose_estimator.cc planar_rgb_model.cc)
target_link_libraries (test_matching ${OpenCV_LIBS})