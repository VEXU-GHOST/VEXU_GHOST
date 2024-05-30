cmake_minimum_required(VERSION 3.8)
project(ghost_sim_examples)

set(Ghost_DIR "$ENV{HOME}/cmake")
  add_compile_options(-Wall -Wextra -Wpedantic)
find_package(Ghost CONFIG REQUIRED)

# find dependencies
find_package(ament_cmake REQUIRED)

set(DEPENDENCIES

  ament_cmake
  )

ament_export_dependencies(${DEPENDENCIES})

#install
install(DIRECTORY
  launch
  urdf
  rviz
  world
  DESTINATION share/${PROJECT_NAME})

ament_package()