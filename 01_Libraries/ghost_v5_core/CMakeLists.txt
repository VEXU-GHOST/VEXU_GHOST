cmake_minimum_required(VERSION 3.8)
project(ghost_v5_core)

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
  yaml-cpp
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

########################
### Filter Libraries ###
########################
add_library(first_order_low_pass_filter SHARED
src/filters/first_order_low_pass_filter.cpp
)
target_include_directories(first_order_low_pass_filter
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_export_targets(first_order_low_pass_filter HAS_LIBRARY_TARGET)
install(
  TARGETS first_order_low_pass_filter
  EXPORT first_order_low_pass_filter
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

add_library(second_order_low_pass_filter SHARED
src/filters/second_order_low_pass_filter.cpp
)
target_include_directories(second_order_low_pass_filter
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_export_targets(second_order_low_pass_filter HAS_LIBRARY_TARGET)
install(
  TARGETS second_order_low_pass_filter
  EXPORT second_order_low_pass_filter
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

#######################
### Motor Libraries ###
#######################
# DC Motor Model Library
add_library(dc_motor_model SHARED src/motor/dc_motor_model.cpp)
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

add_library(motor_controller SHARED src/motor/motor_controller.cpp)
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

#############################
##### Utility Libraries #####
#############################
add_library(load_motor_device_config_yaml SHARED
  src/devices/motor/load_motor_device_config_yaml.cpp
)
target_include_directories(load_motor_device_config_yaml
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(load_motor_device_config_yaml
  ${DEPENDENCIES}
)
ament_export_targets(load_motor_device_config_yaml HAS_LIBRARY_TARGET)
install(
  TARGETS load_motor_device_config_yaml
  EXPORT load_motor_device_config_yaml
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

add_library(load_rotation_sensor_device_config_yaml SHARED
  src/devices/rotation_sensor/load_rotation_sensor_device_config_yaml.cpp
)
target_include_directories(load_rotation_sensor_device_config_yaml
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(load_rotation_sensor_device_config_yaml
  ${DEPENDENCIES}
)
ament_export_targets(load_rotation_sensor_device_config_yaml HAS_LIBRARY_TARGET)
install(
  TARGETS load_rotation_sensor_device_config_yaml
  EXPORT load_rotation_sensor_device_config_yaml
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

add_library(device_config_factory_utils SHARED
  src/devices/device_config_factory_utils.cpp
)
target_include_directories(device_config_factory_utils
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(device_config_factory_utils
  ${DEPENDENCIES}
)
target_link_libraries(device_config_factory_utils
  load_motor_device_config_yaml
  load_rotation_sensor_device_config_yaml
)
ament_export_targets(device_config_factory_utils HAS_LIBRARY_TARGET)
install(
  TARGETS device_config_factory_utils
  EXPORT device_config_factory_utils
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

#################
#### Testing ####
#################
ament_add_gtest(test_dc_motor_model test/motor/test_dc_motor_model.cpp)
target_link_libraries(test_dc_motor_model
  gtest_main
  dc_motor_model
)

ament_add_gtest(test_load_motor_device_config_yaml test/devices/motor/test_load_motor_device_config_yaml.cpp)
ament_target_dependencies(test_load_motor_device_config_yaml ${DEPENDENCIES})
target_link_libraries(test_load_motor_device_config_yaml
  gtest_main
  load_motor_device_config_yaml
  yaml-cpp
)

ament_add_gtest(test_load_rotation_sensor_device_config_yaml test/devices/rotation_sensor/test_load_rotation_sensor_device_config_yaml.cpp)
ament_target_dependencies(test_load_rotation_sensor_device_config_yaml ${DEPENDENCIES})
target_link_libraries(test_load_rotation_sensor_device_config_yaml
  gtest_main
  load_rotation_sensor_device_config_yaml
  yaml-cpp
)

ament_add_gtest(test_device_config_interfaces test/devices/base/test_device_config_interfaces.cpp)
ament_target_dependencies(test_device_config_interfaces ${DEPENDENCIES})
target_link_libraries(test_device_config_interfaces
  gtest_main
)

ament_add_gtest(test_device_config_map_factory test/devices/base/test_device_config_map_factory.cpp)
ament_target_dependencies(test_device_config_map_factory ${DEPENDENCIES})
target_link_libraries(test_device_config_map_factory
  gtest_main
  device_config_factory_utils
  yaml-cpp
)

ament_package()
