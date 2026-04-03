cmake_minimum_required(VERSION 3.8)
project(orthomap_viewer)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

set(deps
  rclcpp
  std_msgs
  nav_msgs
  sensor_msgs
  pcl_conversions
)

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(orthomap_viewer
  src/orthomap_viewer.cpp
)
ament_target_dependencies(orthomap_viewer ${deps})
target_link_libraries(orthomap_viewer
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(TARGETS orthomap_viewer
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
