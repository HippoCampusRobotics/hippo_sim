add_library(BuoyancyPlugin SHARED
  buoyancy.cpp
)

ament_target_dependencies(BuoyancyPlugin
  ignition-plugin${GZ_PLUGIN_VER}
  ignition-gazebo${GZ_GAZEBO_VER}
  ignition-math${GZ_MATH_VER}
)

install(TARGETS BuoyancyPlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
