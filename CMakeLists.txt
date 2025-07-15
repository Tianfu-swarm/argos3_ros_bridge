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
find_package(tf2_ros REQUIRED)


find_package(ARGoS REQUIRED)
find_package(GSL REQUIRED)
find_package(Lua REQUIRED)



include_directories(
  include
  ${ARGOS_INCLUDE_DIRS}
  ${GSL_INCLUDE_DIR}
  ${LUA_INCLUDE_DIR}
)

link_directories(
  ${ARGOS_LIBRARY_DIRS}
)

#######################################
# Plugin 1: Robot controller plugin
#######################################
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

#######################################
# Plugin 2: LoopFunctions plugin
#######################################
add_library(bridge_loop_function SHARED
  src/bridge_loop_function.cpp
)

target_include_directories(bridge_loop_function PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(bridge_loop_function
  ${ARGOS_LDFLAGS}
)

ament_target_dependencies(bridge_loop_function
  rclcpp
  std_msgs
)

install(TARGETS bridge_loop_function
  LIBRARY DESTINATION lib
)

#######################################
# Install headers
#######################################
install(DIRECTORY include/
  DESTINATION include/
)

#######################################
# ament package
#######################################
ament_package()

