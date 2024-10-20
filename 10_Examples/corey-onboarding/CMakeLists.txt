cmake_minimum_required(VERSION 3.8)
project(corey-onboarding)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(bankmaincode
  test/banktest.cpp
)

add_executable(publisher_node src/publisher.cpp)
ament_target_dependencies(publisher_node rclcpp std_msgs)

add_executable(subscriber_node src/subscriber.cpp)
ament_target_dependencies(subscriber_node rclcpp std_msgs)

ament_add_gtest(test_person test_person.cpp)
target_link_libraries(test_person ${GTEST_INCLUDE_DIRS} pthread)

add_executable(hello_world src/helloworld.cpp)
install(TARGETS
  hello_world
  publisher_node
  subscriber_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
