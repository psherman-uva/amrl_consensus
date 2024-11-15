
add_executable(simulation_node 
  simulation_node.cpp
  src/SimulationNode.cpp)
target_link_libraries(simulation_node
  simulation_robot
  ${catkin_LIBRARIES})
