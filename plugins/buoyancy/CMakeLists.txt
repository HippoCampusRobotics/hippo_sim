add_library(BuoyancyPlugin SHARED
  buoyancy.cpp
)

ament_target_dependencies(BuoyancyPlugin
  ignition-plugin${IGN_PLUGIN_VER}
  ignition-gazebo${IGN_GAZEBO_VER}
)

install(TARGETS BuoyancyPlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
