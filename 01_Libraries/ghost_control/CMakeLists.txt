cmake_minimum_required(VERSION 3.8)
project(ghost_control)

# Add Global CMake Config for Ghost Packages
set(Ghost_DIR "$ENV{HOME}/VEXU_GHOST/cmake")
find_package(Ghost CONFIG REQUIRED)

######################
#### Dependencies ####
######################
# Set Package Dependencies
set(DEPENDENCIES
  ament_cmake
  ament_cmake_gtest
  ghost_util
  ghost_estimation
  )

foreach(pkg ${DEPENDENCIES})
  find_package(${pkg} REQUIRED)
endforeach()
ament_export_dependencies(${DEPENDENCIES})

# Set Include Directories
set(INCLUDE
  include
)

include_directories(${INCLUDE})
ament_export_include_directories(${INCLUDE})


#################
### Libraries ###
#################
# DC Motor Model
add_library(dc_motor_model SHARED src/models/dc_motor_model.cpp)
target_include_directories(dc_motor_model
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_export_targets(dc_motor_model HAS_LIBRARY_TARGET)
install(
  TARGETS dc_motor_model
  EXPORT dc_motor_model
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Motor Controller
add_library(motor_controller SHARED src/motor_controller.cpp)
ament_target_dependencies(motor_controller
${DEPENDENCIES}
)
target_include_directories(motor_controller
PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
ament_export_targets(motor_controller HAS_LIBRARY_TARGET)
install(
  TARGETS motor_controller
  EXPORT motor_controller
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

#################
#### Install ####
#################
install(
  DIRECTORY include/
  DESTINATION include
)

###############
### Testing ###
###############
ament_add_gtest(test_dc_motor_model test/test_dc_motor_model.cpp)
target_link_libraries(test_dc_motor_model
  dc_motor_model
  gtest
)

ament_package()
