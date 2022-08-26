cmake_minimum_required(VERSION 3.5)
find_package(ignition-cmake2 REQUIRED)
project(buoyancy-plugin)

find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

find_package(ignition-gazebo6 REQUIRED)
add_library(BuoyancyPlugin SHARED buoyancy.cpp)
set_property(TARGET BuoyancyPlugin PROPERTY CXX_STANDARD 17)
target_link_libraries(BuoyancyPlugin
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo6::ignition-gazebo6)
