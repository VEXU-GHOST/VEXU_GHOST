cmake_minimum_required(VERSION 3.5)
project(ghost_serial)

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
  ghost_msgs
  )

foreach(pkg ${DEPENDENCIES})
  find_package(${pkg} REQUIRED)
endforeach()
ament_export_dependencies(${DEPENDENCIES})

include_directories(include)

####################
### COBS Library ###
####################
add_library(cobs SHARED
  src/cobs/cobs.cpp
)
target_include_directories(cobs
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_export_targets(cobs HAS_LIBRARY_TARGET)
install(
  TARGETS cobs
  EXPORT cobs
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

##################
### Msg Parser ###
##################
add_library(msg_parser SHARED
  src/msg_parser/msg_parser.cpp
)
target_link_libraries(msg_parser
  cobs
)
target_include_directories(msg_parser
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_export_targets(msg_parser HAS_LIBRARY_TARGET)
install(
  TARGETS msg_parser
  EXPORT msg_parser
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

######################
### Serial Library ###
######################
add_library(jetson_serial_base SHARED
  src/base_interfaces/generic_serial_base.cpp
  src/base_interfaces/jetson_serial_base.cpp
)
target_link_libraries(jetson_serial_base
  cobs
  msg_parser
)
target_include_directories(jetson_serial_base
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_export_targets(jetson_serial_base HAS_LIBRARY_TARGET)
install(
  TARGETS jetson_serial_base
  EXPORT jetson_serial_base
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

###############
### Testing ###
###############
ament_add_gtest(test_cobs test/test_cobs.cpp)
target_link_libraries(test_cobs
  cobs
  gtest
)
ament_add_gtest(test_msg_parser test/test_msg_parser.cpp)
target_link_libraries(test_msg_parser
  msg_parser
  gtest
)

###############
### Install ###
###############
install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
