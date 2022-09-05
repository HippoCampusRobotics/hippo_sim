add_library(HydrodynamicsPlugin SHARED
  hydrodynamics.cpp
)

ament_target_dependencies(HydrodynamicsPlugin
  ignition-plugin${IGN_PLUGIN_VER}
  ignition-gazebo${IGN_GAZEBO_VER}
)

install(TARGETS HydrodynamicsPlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

