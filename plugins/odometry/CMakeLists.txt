add_library(OdometryPlugin SHARED
  odometry.cpp
)

ament_target_dependencies(OdometryPlugin
  ignition-plugin${GZ_PLUGIN_VER}
  ignition-gazebo${GZ_GAZEBO_VER}
  ignition-transport${GZ_TRANSPORT_VER}
  ignition-math${GZ_MATH_VER}
)

install(TARGETS OdometryPlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
