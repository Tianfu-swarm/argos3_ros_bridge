cmake_minimum_required(VERSION 3.8)
project(collective_decision_making)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rosidl_default_generators REQUIRED)

find_package(rclcpp REQUIRED)

find_package(geometry_msgs REQUIRED)

find_package(std_msgs REQUIRED)

#find_package(message_generation REQUIRED)


set(msg_files
  "msg/Led.msg"
  "msg/Blob.msg"
  "msg/BlobList.msg"
  "msg/Proximity.msg"
  "msg/ProximityList.msg"
  "msg/Light.msg"
  "msg/LightList.msg"
  "msg/Position.msg"
  "msg/Packet.msg"
  "msg/PacketList.msg"
  
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
    DEPENDENCIES builtin_interfaces std_msgs geometry_msgs

)


  
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

###########
## Build ##
###########

# Added for ARGoS
add_subdirectory(plugins)

#ament_export_include_directories()

ament_package()
