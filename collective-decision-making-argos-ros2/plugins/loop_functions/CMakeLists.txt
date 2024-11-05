find_package(collective_decision_making REQUIRED)
include_directories(${ARGOS_QTOPENGL_INCLUDE_DIRS})
add_library(trajectory_loop_functions MODULE
  util.h
  trajectory_loop_functions.h
  trajectory_qtuser_functions.h
  util.cpp
  trajectory_loop_functions.cpp
  trajectory_qtuser_functions.cpp)
  


target_link_libraries(trajectory_loop_functions
  ${ARGOS_LIBRARY_DIRS}
  ${ARGOS_LDFLAGS}
  ${rclcpp_LIBRARIES}
  /usr/local/lib/argos3/libargos3core_simulator.so
  /usr/local/lib/argos3/libargos3plugin_simulator_entities.so
  /usr/local/lib/argos3/libargos3plugin_simulator_footbot.so
  /usr/local/lib/argos3/libargos3plugin_simulator_qtopengl.so
  ${ARGOS_QTOPENGL_LIBRARIES})
  
ament_target_dependencies( trajectory_loop_functions ARGoS rclcpp std_msgs collective_decision_making )

install(
    TARGETS trajectory_loop_functions
    EXPORT trajectory_loop_functions
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION lib
)
