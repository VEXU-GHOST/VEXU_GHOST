cmake_minimum_required(VERSION 3.8)
project(Safi_onboarding)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

# Library (our class)
add_library(sayHello_lib src/hello_world.cpp)
target_include_directories(sayHello_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(sayHello_lib rclcpp std_msgs)


# Executable 1: hello_world
add_executable(hello_world src/hello_world.cpp)
target_link_libraries(hello_world sayHello_lib)
ament_target_dependencies(hello_world rclcpp std_msgs)

# Executable 2: talker_listener_node
add_executable(talker_listener_node src/talker_listener_node.cpp)
target_link_libraries(talker_listener_node sayHello_lib)
ament_target_dependencies(talker_listener_node rclcpp std_msgs)

# Test
ament_add_gtest(test_hello test/test_hello.cpp)
if(TARGET test_hello)
  target_link_libraries(test_hello sayHello_lib)
endif()

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  #set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()
endif()


#add_executable(hello_world src/hello_world.cpp)
install(TARGETS 
  sayHello_lib
  hello_world
  talker_listener_node
  DESTINATION lib/${PROJECT_NAME})



ament_package()
