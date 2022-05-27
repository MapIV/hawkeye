cmake_minimum_required(VERSION 3.0.2)
project(orthomap_viewer)

set(CMAKE_CXX_STANDARD 17)

find_package(Boost REQUIRED)
find_package(PCL REQUIRED)
find_package(catkin REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  pcl_ros
  sensor_msgs
)

catkin_package(
  #INCLUDE_DIRS include
  DEPENDS PCL
  DEPENDS OpenCV
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(orthomap_viewer
  src/orthomap_viewer.cpp
)
target_link_libraries(orthomap_viewer
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_dependencies(orthomap_viewer ${catkin_EXPORTED_TARGETS})
