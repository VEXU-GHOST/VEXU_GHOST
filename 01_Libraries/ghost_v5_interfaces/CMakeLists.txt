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

################################
### Robot Hardware Interface ###
################################
add_library(robot_hardware_interface SHARED
  src/robot_hardware_interface.cpp
)
target_include_directories(robot_hardware_interface
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(robot_hardware_interface
  ${DEPENDENCIES}
)
ament_export_targets(robot_hardware_interface HAS_LIBRARY_TARGET)
install(
  TARGETS robot_hardware_interface
  EXPORT robot_hardware_interface
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

#############################
##### Utility Libraries #####
#############################
set(UTIL_LIBS
  load_motor_device_config_yaml
  load_rotation_sensor_device_config_yaml
)

foreach(UTIL ${UTIL_LIBS})
  add_library(${UTIL} SHARED
    src/util/${UTIL}.cpp
  )
  target_include_directories(${UTIL}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${UTIL}
    ${DEPENDENCIES}
  )
  ament_export_targets(${UTIL} HAS_LIBRARY_TARGET)
  install(
    TARGETS ${UTIL}
    EXPORT ${UTIL}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
  )
endforeach()

add_library(device_config_factory_utils SHARED
src/util/device_config_factory_utils.cpp
)
target_include_directories(device_config_factory_utils
PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
ament_target_dependencies(device_config_factory_utils
${DEPENDENCIES}
)
target_link_libraries(device_config_factory_utils
  ${UTIL_LIBS}
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
# Device Interface Tests
set(DEVICE_TESTS
  # test_device_config_interfaces
  test_motor_device_interface
  test_rotation_sensor_device_interface
  test_joystick_device_interface
)
foreach(TEST ${DEVICE_TESTS})
  ament_add_gtest(${TEST} test/devices/${TEST}.cpp)
  ament_target_dependencies(${TEST} ${DEPENDENCIES})
  target_link_libraries(${TEST} 
    gtest_main
  )
endforeach()

# Utility Tests
foreach(TEST ${UTIL_LIBS})
  ament_add_gtest(test_${TEST} test/util/test_${TEST}.cpp)
  ament_target_dependencies(test_${TEST} ${DEPENDENCIES})
  target_link_libraries(test_${TEST}
    gtest_main
    load_motor_device_config_yaml
    load_rotation_sensor_device_config_yaml
    device_config_factory_utils
    yaml-cpp
  )
endforeach()

# Hardware Interface Tests
ament_add_gtest(test_robot_hardware_interface test/test_robot_hardware_interface.cpp)
ament_target_dependencies(test_robot_hardware_interface ${DEPENDENCIES})
target_link_libraries(test_robot_hardware_interface
  gtest_main
  device_config_factory_utils
  robot_hardware_interface
)

ament_package()
