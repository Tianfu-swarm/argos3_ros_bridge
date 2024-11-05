find_package(collective_decision_making REQUIRED)

add_library(argos_ros_bot argos_ros_footbot.h argos_ros_footbot.cpp)

target_link_libraries(argos_ros_bot
  ${cpp_typesupport_target}
  ${ARGOS_LIBRARY_DIRS}
  ${ARGOS_LDFLAGS}
  ${rclcpp_LIBRARIES}
  /usr/local/lib/argos3/libargos3plugin_simulator_footbot.so
  /usr/local/lib/argos3/libargos3plugin_simulator_genericrobot.so
  /usr/local/lib/argos3/libargos3core_simulator.so)
  
ament_export_dependencies(rosidl_default_runtime rclcpp std_msgs message_runtime collective_decision_making)

ament_target_dependencies(argos_ros_bot rclcpp geometry_msgs std_msgs ARGoS collective_decision_making)

install(
    TARGETS argos_ros_bot
    EXPORT argos_ros_bot
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION lib
)

