cmake_minimum_required(VERSION 3.5)
project(hippo_sim)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})
find_package(ignition-gazebo6 REQUIRED)
set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})

install(
  DIRECTORY launch meshes urdf world textures
  DESTINATION share/${PROJECT_NAME}
)

include_directories(plugin/include)
add_library(BuoyancyPlugin SHARED
  plugin/buoyancy.cpp
)

# target_link_libraries(BuoyancyPlugin
#   ignition-common4::ignition-common4
#   ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
#   ignition-gazebo${IGN_GAZEBO_VER}::core
# )
ament_target_dependencies(BuoyancyPlugin
  ignition-plugin${IGN_PLUGIN_VER}
  ignition-gazebo${IGN_GAZEBO_VER}
)

# ament_target_dependencies(BuoyancyPlugin
#   rclcpp
# )

# ament_export_targets(BuoyancyPlugin HAS_LIBRARY_TARGET)

install(TARGETS BuoyancyPlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
