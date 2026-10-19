cmake_minimum_required(VERSION 3.8)
project(amy_onboarding)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)

add_library(calculator
  src/calculator.cpp
)

add_executable(amy_main
  src/main.cpp
)

target_link_libraries(amy_main
  calculator
)

install(
  TARGETS
    calculator
    amy_main
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

find_package(ament_cmake_gtest REQUIRED)

if(BUILD_TESTING)
  ament_add_gtest(test_calculator
    test/test_calculator.cpp
  )

  target_link_libraries(test_calculator
    calculator
  )
endif()

add_executable(amy_pubsub_node
  src/pubsub_node.cpp
)

ament_target_dependencies(amy_pubsub_node
  rclcpp
  std_msgs
)

install(
  TARGETS amy_pubsub_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
