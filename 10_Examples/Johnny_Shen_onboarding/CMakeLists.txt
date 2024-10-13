
cmake_minimum_required(VERSION 3.8)
project(Johnny_Shen_onboarding)

include_directories(include)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

ament_package()

add_library(hello_world src/hello_world.cpp)
install(TARGETS
  hello_world
  DESTINATION lib/${PROJECT_NAME})
  

add_executable(publish_test src/a.cpp)

ament_target_dependencies(publish_test 
std_msgs
rclcpp)

install(TARGETS
  publish_test
  DESTINATION lib/${PROJECT_NAME})

add_executable(subscribe_test src/b.cpp)

ament_target_dependencies(subscribe_test 
std_msgs
rclcpp)

install(TARGETS
  subscribe_test
  DESTINATION lib/${PROJECT_NAME})

ament_add_gtest(testadd test/c.cpp)
target_link_libraries(testadd
gtest 
hello_world)



