cmake_minimum_required(VERSION 3.5)
project(ghost_serial)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(CMAKE_BUILD_TYPE "DEBUG")

if(CMAKE_BUILD_TYPE EQUAL "DEBUG")
  add_compile_options(-g -O0)
endif(CMAKE_BUILD_TYPE EQUAL "DEBUG")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

find_package(rclcpp REQUIRED)
find_package(ghost_msgs REQUIRED)

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
add_library(serial_interface SHARED
  src/serial_interface/serial_interface.cpp
)
target_link_libraries(serial_interface
  cobs
  msg_parser
)
target_include_directories(serial_interface
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_export_targets(serial_interface HAS_LIBRARY_TARGET)
install(
  TARGETS serial_interface
  EXPORT serial_interface
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

######################
### GHOST ROS Node ###
######################
add_library(v5_serial_node SHARED
  src/v5_serial_node.cpp
)
ament_target_dependencies(v5_serial_node
  rclcpp
  ghost_msgs
)
target_link_libraries(v5_serial_node
  serial_interface
  cobs
  yaml-cpp
)
target_include_directories(v5_serial_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_export_targets(v5_serial_node HAS_LIBRARY_TARGET)
ament_export_dependencies(ghost_msgs)
install(
  TARGETS v5_serial_node
  EXPORT v5_serial_node
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
### Headers ###
###############
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()
