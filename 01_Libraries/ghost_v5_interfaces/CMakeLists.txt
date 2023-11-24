cmake_minimum_required(VERSION 3.8)
project(ghost_v5_interfaces)

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
  ghost_control
  ghost_estimation
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

#############################
##### Utility Libraries #####
#############################
add_library(load_motor_device_config_yaml SHARED
  src/motor/load_motor_device_config_yaml.cpp
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
  src/rotation_sensor/load_rotation_sensor_device_config_yaml.cpp
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
  src/device_config_factory_utils.cpp
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
ament_add_gtest(test_load_motor_device_config_yaml test/motor/test_load_motor_device_config_yaml.cpp)
ament_target_dependencies(test_load_motor_device_config_yaml ${DEPENDENCIES})
target_link_libraries(test_load_motor_device_config_yaml
  gtest_main
  load_motor_device_config_yaml
  yaml-cpp
)

ament_add_gtest(test_load_rotation_sensor_device_config_yaml test/rotation_sensor/test_load_rotation_sensor_device_config_yaml.cpp)
ament_target_dependencies(test_load_rotation_sensor_device_config_yaml ${DEPENDENCIES})
target_link_libraries(test_load_rotation_sensor_device_config_yaml
  gtest_main
  load_rotation_sensor_device_config_yaml
  yaml-cpp
)

ament_add_gtest(test_device_config_interfaces test/base/test_device_config_interfaces.cpp)
ament_target_dependencies(test_device_config_interfaces ${DEPENDENCIES})
target_link_libraries(test_device_config_interfaces
  gtest_main
)

ament_add_gtest(test_device_config_map_factory test/base/test_device_config_map_factory.cpp)
ament_target_dependencies(test_device_config_map_factory ${DEPENDENCIES})
target_link_libraries(test_device_config_map_factory
  gtest_main
  device_config_factory_utils
  yaml-cpp
)

ament_add_gtest(test_motor_device_interface test/motor/test_motor_device_interface.cpp)
ament_target_dependencies(test_motor_device_interface ${DEPENDENCIES})
target_link_libraries(test_motor_device_interface
  gtest_main
)

ament_add_gtest(test_rotation_sensor_device_interface test/rotation_sensor/test_rotation_sensor_device_interface.cpp)
ament_target_dependencies(test_rotation_sensor_device_interface ${DEPENDENCIES})
target_link_libraries(test_rotation_sensor_device_interface
  gtest_main
)

ament_package()
