cmake_minimum_required(VERSION 3.5)
project(hippo_sim)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gflags REQUIRED)

find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})
find_package(ignition-gazebo6 REQUIRED)
set(GZ_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})
find_package(ignition-msgs8 REQUIRED)
set(GZ_MSGS_VER ${ignition-msgs8_VERSION_MAJOR})
find_package(ignition-math6 REQUIRED)
set(GZ_MATH_VER ${ignition-math6_VERSION_MAJOR})
find_package(ignition-transport11 REQUIRED)
set(GZ_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})

add_subdirectory(plugins/buoyancy)
add_subdirectory(plugins/hydrodynamics)
add_subdirectory(plugins/thruster)

add_executable(spawn src/spawn.cpp)
ament_target_dependencies(spawn
  rclcpp
)
target_link_libraries(spawn
  gflags
  ignition-math${GZ_MATH_VER}::core
  ignition-msgs${GZ_MSGS_VER}::core
  ignition-transport${GZ_TRANSPORT_VER}::core
)

install(TARGETS
  spawn
  DESTINATION lib/${PROJECT_NAME}
)

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
