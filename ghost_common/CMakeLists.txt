cmake_minimum_required(VERSION 3.8)
project(ghost_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

######################
#### Dependencies ####
######################
# find dependencies
find_package(ament_cmake REQUIRED)

###############################
##### Robot Configuration #####
###############################
add_library(v5_robot_config SHARED
  src/v5_robot_config_defs.cpp
  src/v5_robot_config.cpp
)
target_include_directories(v5_robot_config
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_export_targets(v5_robot_config HAS_LIBRARY_TARGET)
install(
  TARGETS v5_robot_config
  EXPORT v5_robot_config
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


ament_package()
