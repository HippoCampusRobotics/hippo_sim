add_library(ThrusterPlugin SHARED
  thruster.cpp
)

ament_target_dependencies(ThrusterPlugin
  ignition-plugin${IGN_PLUGIN_VER}
  ignition-gazebo${IGN_GAZEBO_VER}
)

install(TARGETS ThrusterPlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
