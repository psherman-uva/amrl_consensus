
add_executable(consensus_node 
  consensus_node.cpp
  src/ConsensusNode.cpp)
target_link_libraries(consensus_node
amrl_consensus
amrl_joystick
${catkin_LIBRARIES})

add_dependencies(consensus_node amrl_logging_generate_messages amrl_display_generate_messages)
