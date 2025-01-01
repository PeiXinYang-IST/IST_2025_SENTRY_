cmake_minimum_required(VERSION 2.8.3)
project(fast_lio_localization)

SET(CMAKE_BUILD_TYPE "Debug")

ADD_COMPILE_OPTIONS(-std=c++14 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set( CMAKE_CXX_FLAGS "-std=c++14 -O3" )

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -std=c++0x -std=c++14 -fexceptions")

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=1)
    message("core for MP: 1")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()


find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(PythonLibs REQUIRED)
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf2_ros
  tf
  tf_conversions
  livox_ros_driver2
  message_generation
  eigen_conversions
  pcl_conversions
  fast_lio
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)

message(Eigen: ${EIGEN3_INCLUDE_DIR})

include_directories(
	${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  include)

add_message_files(
  FILES
  Pose6D.msg
)

generate_messages(
 DEPENDENCIES
 geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS
)

add_executable(fastlio_mapping1 src/laserMapping.cpp include/ikd-Tree/ikd_Tree.cpp src/preprocess.cpp)
add_executable(pcl_processing_node src/pcl_processing_node.cpp)
add_executable(ICP_location src/ICP_location.cpp)
add_executable(MY_ICP src/MY_ICP.cpp)
add_executable(point_deal src/point_deal.cpp)
add_executable(Obstacle_cloudget src/Obstacle_cloudget.cpp)
add_executable(sim_obstacle_map_cloud src/sim_obstacle_map_cloud.cpp)
add_executable(sim_point_cloud_deal src/sim_point_cloud_deal.cpp)
# add_executable(small_gicp src/small_gicp.cpp)
target_link_libraries(pcl_processing_node ${catkin_LIBRARIES})
target_link_libraries(ICP_location ${catkin_LIBRARIES})
target_link_libraries(sim_obstacle_map_cloud ${catkin_LIBRARIES})
target_link_libraries(sim_point_cloud_deal ${catkin_LIBRARIES})
# target_link_libraries(small_gicp ${catkin_LIBRARIES})
target_link_libraries(MY_ICP ${catkin_LIBRARIES})
target_link_libraries(point_deal ${catkin_LIBRARIES})
target_link_libraries(Obstacle_cloudget ${catkin_LIBRARIES})
target_link_libraries(fastlio_mapping1 ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES})
target_include_directories(fastlio_mapping1 PRIVATE ${PYTHON_INCLUDE_DIRS})

