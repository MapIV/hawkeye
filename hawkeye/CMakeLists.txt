cmake_minimum_required(VERSION 3.0.2)
project(hawkeye)

set(CMAKE_CXX_STANDARD 17)

find_package(Boost REQUIRED)
find_package(PCL REQUIRED)
find_package(catkin REQUIRED)
find_package(OpenCV REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_definitions(${OpenCV_DEFINITIONS})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  pcl_ros
  sensor_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  cv_bridge
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL
  DEPENDS OpenCV
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

set(srcs 
  src/map/orthomap.cpp
  src/map/orthomap_read.cpp
  src/util/tf_messages.cpp
  src/util/pose_stamped_helper.cpp
)

set(hawkeye_srcs
  src/hawkeye_base/base.cpp
  src/hawkeye_base/node.cpp
  src/hawkeye_base/config.cpp
)

# hawkeye_rt
add_executable(hawkeye_rt
  ${srcs}
  ${hawkeye_srcs}
  src/hawkeye_rt/hawkeye_node.cpp
)
target_link_libraries(hawkeye_rt
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)
add_dependencies(hawkeye_rt ${catkin_EXPORTED_TARGETS})

# hawkeye_rt_ws
add_executable(hawkeye_rt_ws
  ${srcs}
  ${hawkeye_srcs}
  src/hawkeye_rt/hawkeye_ws_node.cpp
)
target_link_libraries(hawkeye_rt_ws
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)
add_dependencies(hawkeye_rt_ws ${catkin_EXPORTED_TARGETS})
