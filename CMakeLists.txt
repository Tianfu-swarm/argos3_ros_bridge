cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0072 NEW)

project(argos3_ros_bridge)

# Compile settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ARGoS REQUIRED)
find_package(GSL REQUIRED)
find_package(Lua REQUIRED)
find_package(tf2_ros REQUIRED)


include_directories(
  include
  ${ARGOS_INCLUDE_DIRS}
  ${GSL_INCLUDE_DIR}
  ${LUA_INCLUDE_DIR}
)

link_directories(
  ${ARGOS_LIBRARY_DIRS}
)

# Build the plugin as shared library
add_library(argos_ros_bot SHARED
  src/argos_ros_footbot.cpp
)

target_include_directories(argos_ros_bot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(argos_ros_bot
  ${ARGOS_LDFLAGS}
)

ament_target_dependencies(argos_ros_bot
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
)

install(TARGETS argos_ros_bot
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()

