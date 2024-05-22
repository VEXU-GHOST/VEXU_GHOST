cmake_minimum_required(VERSION 3.8)
project(ghost_ros_interfaces)

# Add Global CMake Config for Ghost Packages
set(Ghost_DIR "$ENV{VEXU_HOME}/cmake")
find_package(Ghost CONFIG REQUIRED)

######################
#### Dependencies ####
######################
# Set Package Dependencies
set(DEPENDENCIES
  ament_cmake
  ament_cmake_gtest
  ghost_v5_interfaces
  ghost_msgs
  ghost_serial
  ghost_util
  ghost_planners
  pluginlib
  rclcpp
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

############################
#### Msg Helper Library ####
############################
add_library(msg_helpers SHARED
src/msg_helpers/msg_helpers.cpp
)
target_include_directories(msg_helpers
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(msg_helpers
  ${DEPENDENCIES}
)
ament_export_targets(msg_helpers HAS_LIBRARY_TARGET)
install(
  TARGETS msg_helpers
  EXPORT msg_helpers
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

#####################
#### Executables ####
#####################
# Jetson Serial Node
add_executable(jetson_v5_serial_node
  src/serial/jetson_v5_serial_node.cpp
)
ament_target_dependencies(jetson_v5_serial_node
  ${DEPENDENCIES}
)
target_link_libraries(jetson_v5_serial_node
  ghost_serial::jetson_serial_base
  ghost_serial::cobs
  msg_helpers
  yaml-cpp
)
target_include_directories(jetson_v5_serial_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS
  jetson_v5_serial_node
  DESTINATION lib/${PROJECT_NAME})

# V5 Robot Base Library
add_library(v5_robot_base SHARED src/competition/v5_robot_base.cpp)
target_include_directories(v5_robot_base
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(v5_robot_base
  ${DEPENDENCIES}
)
target_link_libraries(v5_robot_base
  msg_helpers
  )
ament_export_targets(v5_robot_base HAS_LIBRARY_TARGET)
install(
  TARGETS v5_robot_base
  EXPORT v5_robot_base
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Timer Service
add_executable(timer_service
  src/timer/timer_service.cpp
)
ament_target_dependencies(timer_service
  ${DEPENDENCIES}
)
target_include_directories(timer_service
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS
  timer_service
  DESTINATION lib/${PROJECT_NAME})
  
add_executable(start_timer_client
  src/timer/start_timer_client.cpp
)
ament_target_dependencies(start_timer_client
  ${DEPENDENCIES}
)
target_include_directories(start_timer_client
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS
  start_timer_client
  DESTINATION lib/${PROJECT_NAME})
  
add_executable(check_timer_client
  src/timer/check_timer_client.cpp
)
ament_target_dependencies(check_timer_client
  ${DEPENDENCIES}
)
target_include_directories(check_timer_client
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS
  check_timer_client
  DESTINATION lib/${PROJECT_NAME})


# Competition State Machine Node
add_executable(competition_state_machine_node
  src/competition/competition_state_machine_node.cpp
)
ament_target_dependencies(competition_state_machine_node
  ${DEPENDENCIES}
)
target_link_libraries(competition_state_machine_node
  v5_robot_base
  yaml-cpp
)
target_include_directories(competition_state_machine_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS
  competition_state_machine_node
  DESTINATION lib/${PROJECT_NAME})

# Bag Recorder Nodes
add_executable(bag_recorder_service
  src/bag_recorder/bag_recorder_service.cpp
)
ament_target_dependencies(bag_recorder_service
  ${DEPENDENCIES}
)
target_include_directories(bag_recorder_service
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS
  bag_recorder_service
  DESTINATION lib/${PROJECT_NAME})
  
add_executable(start_bag_recorder_client
  src/bag_recorder/start_bag_recorder_client.cpp
)
ament_target_dependencies(start_bag_recorder_client
  ${DEPENDENCIES}
)
target_include_directories(start_bag_recorder_client
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS
  start_bag_recorder_client
  DESTINATION lib/${PROJECT_NAME})
  
add_executable(stop_bag_recorder_client
  src/bag_recorder/stop_bag_recorder_client.cpp
)
ament_target_dependencies(stop_bag_recorder_client
  ${DEPENDENCIES}
)
target_include_directories(stop_bag_recorder_client
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS
  stop_bag_recorder_client
  DESTINATION lib/${PROJECT_NAME})
  
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
set(TESTS
  test_robot_hardware_interface_ros
  test_device_interfaces_ros
  test_msg_helpers
)
foreach(TEST ${TESTS})
  ament_add_gtest(${TEST} test/${TEST}.cpp)
  ament_target_dependencies(${TEST} ${DEPENDENCIES})
  target_link_libraries(${TEST}
    gtest_main
    msg_helpers
  )
endforeach()

ament_package()
