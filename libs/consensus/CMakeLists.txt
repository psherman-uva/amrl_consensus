
add_library(consensus
  src/FormationConsensus.cpp)
target_link_libraries(consensus
  runge_kutta)
