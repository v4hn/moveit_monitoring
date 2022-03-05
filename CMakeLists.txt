cmake_minimum_required(VERSION 3.0.2)
project(moveit_monitoring)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  moveit_core
  moveit_ros_move_group
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
   src/monitor_capability.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES ${PROJECT_NAME}_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
