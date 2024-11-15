add_library(robot_interface
  src/RobotInterface.cpp)
target_link_libraries(robot_interface
  ${catkin_LIBRARIES})