cmake_minimum_required(VERSION 3.8)
project(ghost_v5_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

######################
#### Dependencies ####
######################
# Set Package Dependencies
set(DEPENDENCIES
  ament_cmake
  ament_cmake_gtest
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

#########################
### Device Interfaces ###
#########################

#############################
##### Utility Libraries #####
#############################
add_library(load_config_yaml SHARED
  src/utils/load_config_yaml.cpp
)
target_include_directories(load_config_yaml
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(load_config_yaml
  ${DEPENDENCIES}
)
ament_export_targets(load_config_yaml HAS_LIBRARY_TARGET)
install(
  TARGETS load_config_yaml
  EXPORT load_config_yaml
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
# ament_add_gtest(test_interface_constructors test/test_interface_constructors.cpp)
# ament_target_dependencies(test_interface_constructors ${DEPENDENCIES})
# target_link_libraries(test_interface_constructors
#   gtest_main
# )

ament_add_gtest(test_dc_motor_model test/motor/test_dc_motor_model.cpp)
target_link_libraries(test_dc_motor_model
  gtest_main
  dc_motor_model
)

ament_add_gtest(test_load_config_yaml test/utils/test_load_config_yaml.cpp)
ament_target_dependencies(test_load_config_yaml ${DEPENDENCIES})
target_link_libraries(test_load_config_yaml
  gtest_main
  load_config_yaml
  yaml-cpp
)

ament_package()
