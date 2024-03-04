cmake_minimum_required(VERSION 3.8)
project(satvik-onboarding)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)


# Set Include Directories
set(INCLUDE
  include
)
set(DEPENDENCIES
  ament_cmake
  rclcpp
  geometry_msgs
  )

include_directories(${INCLUDE})

add_executable(trivial_main src/trivial_main.cpp)
install(TARGETS
  trivial_main
  DESTINATION lib/${PROJECT_NAME})

add_executable(test_ps src/test_ps.cpp)
ament_target_dependencies(test_ps geometry_msgs)
ament_target_dependencies(test_ps rclcpp)
install(TARGETS
  test_ps
  DESTINATION lib/${PROJECT_NAME})

add_library(ros_subscriber_example SHARED src/ros_subscriber_example.cpp)
ament_target_dependencies(ros_subscriber_example
  ${DEPENDENCIES}
)
ament_export_targets(ros_subscriber_example HAS_LIBRARY_TARGET)
install(
  TARGETS ros_subscriber_example
  EXPORT ros_subscriber_example
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)


ament_package()
