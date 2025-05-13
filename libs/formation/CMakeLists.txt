
add_library(formation_definitions
  src/Formation.cpp
  src/FormationFromFile.cpp
  src/FormationCircle.cpp)

add_library(formation_supervisor
  src/FormationSupervisor.cpp)
target_link_libraries(formation_supervisor
  formation_definitions)
  