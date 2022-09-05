add_library(HydrodynamicsPlugin SHARED
  hydrodynamics.cpp
)

ament_target_dependencies(HydrodynamicsPlugin
  ignition-plugin${GZ_PLUGIN_VER}
  ignition-gazebo${GZ_GAZEBO_VER}
  ignition-math${GZ_MATH_VER}
)

install(TARGETS HydrodynamicsPlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

