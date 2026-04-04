cmake_minimum_required(VERSION 3.8)
project(hawkeye)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(deps
  rclcpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
  cv_bridge
)

include_directories(include
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
ament_target_dependencies(hawkeye_rt ${deps})
target_link_libraries(hawkeye_rt
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

# hawkeye_rt_ws
add_executable(hawkeye_rt_ws
  ${srcs}
  ${hawkeye_srcs}
  src/hawkeye_rt/hawkeye_ws_node.cpp
)
ament_target_dependencies(hawkeye_rt_ws ${deps})
target_link_libraries(hawkeye_rt_ws
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

add_library(${PROJECT_NAME}_lib
  ${srcs}
  ${hawkeye_srcs}
)
ament_target_dependencies(${PROJECT_NAME}_lib ${deps})
target_link_libraries(${PROJECT_NAME}_lib
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

install(TARGETS hawkeye_rt hawkeye_rt_ws
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
