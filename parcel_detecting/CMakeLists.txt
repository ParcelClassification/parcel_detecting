cmake_minimum_required(VERSION 3.0.2)
project(parcel_detecting)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  parcel_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  tf2
  tf2_ros
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES parcel_detecting opencv tf parcel_msgs
  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport roscpp sensor_msgs std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

add_executable(parcel_main src/parcel_main.cpp)
add_dependencies(parcel_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(parcel_main
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
)