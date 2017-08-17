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
link_directories(/home/user/gurobi702/linux64/lib)

add_definitions(${PCL_DEFINITIONS} -std=c++11 -g -fext-numeric-literals)

#MESSAGE(STATUS "PCL_INCLUDE_DIR: " ${PCL_INCLUDE_DIRS})
#MESSAGE(STATUS "PCL_LIB_DIR: " ${PCL_LIBRARY_DIRS})
MESSAGE(STATUS "OpenCV_INCLUDE_DIR" ${OpenCV_INCLUDE_DIRS})

add_executable (kuka_camera_calibrate kuka_camera_calibrate.cc)
target_link_libraries (kuka_camera_calibrate ${PCL_LIBRARIES} ${OpenCV_LIBS} apriltags drake::drake drake::drake-lcmtypes-cpp)

#add_executable (jjz_controller_main jjz_controller_main.cc)
#target_link_libraries (jjz_controller_main drake::drake drake::drake-lcmtypes-cpp #libgurobi70.so)

#add_executable (pcl_save_pcd pcl_save_pcd.cc)
#target_link_libraries (pcl_save_pcd ${PCL_LIBRARIES} drake::drake)

#add_executable (test_openni_comm openni_comm.cc)
#target_link_libraries (test_openni_comm ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable (demo_visualize demo_visualize_fused_cloud.cc perception.cc point_cloud_registration.cc)
target_link_libraries (demo_visualize ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_library(lib_perception perception.cc point_cloud_registration.cc openni_comm.cc)
target_link_libraries(lib_perception ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable (test_hand_eye hand_eye.cc)
target_link_libraries(test_hand_eye lib_perception drake::drake ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable (robot_cmd_tool robot_cmd_tool.cc)
target_link_libraries(robot_cmd_tool drake::drake)

add_executable (calibration_hand_eye calibration_hand_eye.cc openni_comm.cc)
target_link_libraries(calibration_hand_eye ${PCL_LIBRARIES} ${OpenCV_LIBS} apriltags drake::drake)

#add_library(lib_hand_eye hand_eye.cc perception.cc point_cloud_registration.cc openni_comm.cc)
#target_link_libraries(lib_hand_eye drake::drake ${PCL_LIBRARIES} ${OpenCV_LIBS})

#add_executable (test_fusion action_scan.cc hand_eye.cc perception.cc point_cloud_registration.cc openni_comm.cc)
#target_link_libraries(test_fusion drake::drake ${PCL_LIBRARIES} ${OpenCV_LIBS})

