
add_library(consensus_logging 
  src/ConsensusLogging.cpp)
target_link_libraries(consensus_logging
  amrl_logging_util
  ${catkin_LIBRARIES})
add_dependencies(consensus_logging amrl_logging_generate_messages)