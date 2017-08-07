cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(openni_grabber)

set(drake_DIR "/home/user/drake_bazel_build/lib/cmake/drake")
#list(APPEND CMAKE_MODULE_PATH "/home/user/opencv/build/")
set(CMAKE_MODULE_PATH "/home/user/opencv/build/;${CMAKE_MODULE_PATH}")

MESSAGE(STATUS "CMAKE MODULE PATH" ${CMAKE_MODULE_PATH})
find_package(PCL 1.8 REQUIRED)
#find_package(OpenCV 2.4 REQUIRED)
find_package(OpenCV 2.4.8 EXACT REQUIRED)
#find_package(OpenCV 3.3.0 EXACT REQUIRED PATHS /home/user/opencv/build)
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

#MESSAGE(STATUS "PCL_INCLUDE_DIR: " ${PCL_INCLUDE_DIRS})
#MESSAGE(STATUS "PCL_LIB_DIR: " ${PCL_LIBRARY_DIRS})
MESSAGE(STATUS "OpenCV_INCLUDE_DIR" ${OpenCV_INCLUDE_DIRS})

add_executable (kuka_camera_calibrate kuka_camera_calibrate.cc)
target_link_libraries (kuka_camera_calibrate ${PCL_LIBRARIES} ${OpenCV_LIBS} apriltags drake::drake drake::drake-lcmtypes-cpp)

#add_executable (pcl_save_pcd pcl_save_pcd.cc)
#target_link_libraries (pcl_save_pcd ${PCL_LIBRARIES} drake::drake)

add_executable (test_perception_interface perception.cc point_cloud_registration.cc)
target_link_libraries (test_perception_interface ${PCL_LIBRARIES} drake::drake ${OpenCV_LIBS})

#add_executable (test_planar_tracking planar_rgb_pose_estimation.cc)
#target_link_libraries (test_planar_tracking ${OpenCV_LIBS})

add_executable (test_openni_comm openni_comm.cc)
target_link_libraries (test_openni_comm ${OpenCV_LIBS} ${PCL_LIBRARIES})