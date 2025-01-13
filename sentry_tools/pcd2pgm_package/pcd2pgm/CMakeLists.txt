cmake_minimum_required(VERSION 3.0.2)
project(pcd2pgm)

# if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
#   include(ProcessorCount)
#   ProcessorCount(N)
#   message("Processer number:  ${N}")
#   if(N GREATER 4)
#     add_definitions(-DMP_EN)
#     add_definitions(-DMP_PROC_NUM=2)
#     message("core for MP: 2")
#   elseif(N GREATER 3)
#     add_definitions(-DMP_EN)
#     add_definitions(-DMP_PROC_NUM=2)
#     message("core for MP: 2")
#   else()
#     add_definitions(-DMP_PROC_NUM=1)
#   endif()
# else()
#   add_definitions(-DMP_PROC_NUM=1)
# endif()

## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  pcl_ros
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pcd2pgm
#  CATKIN_DEPENDS nav_msgs pcl roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


find_package(PCL REQUIRED)

include_directories(
  include
		${PCL_INCLUDE_DIRS})


add_executable(pcd2pgm src/pcd2pgm.cpp)
target_link_libraries(pcd2pgm ${catkin_LIBRARIES} ${PCL_LIBRARIES}   )


