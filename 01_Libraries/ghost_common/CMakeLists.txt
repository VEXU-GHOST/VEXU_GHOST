cmake_minimum_required(VERSION 3.8)
project(ghost_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
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

###############################
##### Robot Configuration #####
###############################
# add_library(v5_robot_config SHARED
#   src/v5_robot_config_defs.cpp
#   src/v5_robot_config.cpp
# )
# target_include_directories(v5_robot_config
#   PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# ament_export_targets(v5_robot_config HAS_LIBRARY_TARGET)
# install(
#   TARGETS v5_robot_config
#   EXPORT v5_robot_config
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
#   RUNTIME DESTINATION bin
#   INCLUDES DESTINATION include
#   )

######################
##### Ghost Util #####
######################
add_library(angle_util SHARED
  src/util/angle_util.cpp
)
target_include_directories(angle_util
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_export_targets(angle_util HAS_LIBRARY_TARGET)
install(
  TARGETS angle_util
  EXPORT angle_util
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

add_library(load_robot_config_utils SHARED
  src/util/load_robot_config.cpp
)
target_include_directories(load_robot_config_utils
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(load_robot_config_utils
  ${DEPENDENCIES}
)
ament_export_targets(load_robot_config_utils HAS_LIBRARY_TARGET)
install(
  TARGETS load_robot_config_utils
  EXPORT load_robot_config_utils
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
ament_add_gtest(test_load_config_yaml test/interfaces/test_load_config_yaml.cpp)
ament_target_dependencies(test_load_config_yaml ${DEPENDENCIES})
target_link_libraries(test_load_config_yaml
  load_robot_config_utils
  yaml-cpp
  gtest
)

ament_package()
