add_library(PosePlugin SHARED
  pose.cpp
)

ament_target_dependencies(PosePlugin
  ignition-plugin${GZ_PLUGIN_VER}
  ignition-gazebo${GZ_GAZEBO_VER}
  ignition-transport${GZ_TRANSPORT_VER}
  ignition-math${GZ_MATH_VER}
)

install(TARGETS PosePlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
