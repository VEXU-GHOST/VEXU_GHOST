cmake_minimum_required(VERSION 3.8)
project(kyle_onboarding)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Add the executable
add_executable(hello_world src/hello_world.cpp)
install(TARGETS
  hello_world
  DESTINATION lib/${PROJECT_NAME})

add_executable(helloworld_node src/helloworld_node.cpp)
ament_target_dependencies(helloworld_node rclcpp std_msgs)
  
install(TARGETS
  helloworld_node
  DESTINATION lib/${PROJECT_NAME}
)

# Enable testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  # Add GTest executable
  ament_add_gtest(test_helloworld test/test_helloworld.cpp)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()