cmake_minimum_required(VERSION 2.8.3)

project(gcopter)

set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)
find_package(CGAL REQUIRED COMPONENTS Core)
find_package(GMP REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  message_generation
  decomp_ros_utils
  decomp_ros_msgs
  decomp_util
  pcl_ros
  pcl_conversions
  cv_bridge
  bspline
  tf2
)

add_message_files(
  FILES
  CoeffRow.msg
  CoeffMatrix.msg
  PolyTrajectory.msg
  Bspline.msg
)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)  # 关键步骤：生成消息头文件
include(${CGAL_USE_FILE})
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../mpc_track_fastplanner/src/Fast-Planner/fast_planner/plan_env/include
    ${OpenCV_INCLUDE_DIRS}
    /home/rm-nuc13/IST_sentry_navigation/src/IST_2025_sentry/mpc_track_fastplanner/src/Fast-Planner/fast_planner/mpc_tracking/include/control.h
)

# catkin_package()
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sentry_serial
#  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs serial std_msgs tf tf2 tf2_ros
#  DEPENDS system_lib
   CATKIN_DEPENDS roscpp std_msgs message_runtime decomp_ros_utils decomp_ros_msgs decomp_util tf tf2
)

add_executable(global_planning src/global_planning.cpp )

target_link_libraries(global_planning
  ${OMPL_LIBRARIES}
  ${catkin_LIBRARIES}
  CGAL::CGAL
  plan_env
)
