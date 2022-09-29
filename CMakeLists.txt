cmake_minimum_required(VERSION 3.5)
project(hippo_sim)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gflags REQUIRED)
find_package(ros_ign_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(hippo_msgs REQUIRED)

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
add_subdirectory(plugins/pose)
add_subdirectory(plugins/odometry)

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

add_executable(bridge src/static_bridge.cpp)
ament_target_dependencies(bridge
  rclcpp
  ros_ign_bridge
  ignition-transport${GZ_TRANSPORT_VER}
  geometry_msgs
  nav_msgs
  hippo_msgs
)

install(TARGETS bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch models
  DESTINATION share/${PROJECT_NAME}
)

# Install Python executables
install(PROGRAMS
  scripts/create_robot_description.py
  scripts/generate_tag_poses.py
  scripts/generate_pool.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
