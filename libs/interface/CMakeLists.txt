add_library(robot_interface
  src/RobotInterface.cpp)
target_link_libraries(robot_interface
  formation_supervisor
  formation_display
  ${catkin_LIBRARIES})