cmake_minimum_required(VERSION 3.5)
project(hippo_sim)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})
find_package(ignition-gazebo6 REQUIRED)
set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})

add_subdirectory(plugins/buoyancy)
add_subdirectory(plugins/hydrodynamics)
add_subdirectory(plugins/thruster)

install(
  DIRECTORY launch meshes urdf world textures
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
