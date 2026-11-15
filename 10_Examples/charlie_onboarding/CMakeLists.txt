cmake_minimum_required(VERSION 3.8)
project(charlie_onboarding)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


add_library(duck_lib STATIC src/duck.cpp)
target_include_directories(duck_lib PUBLIC include)

add_executable(duck src/duck_main.cpp)
target_link_libraries(duck duck_lib)
install(TARGETS duck DESTINATION lib/${PROJECT_NAME})

add_library(long_amogus_lib STATIC src/long_amogus.cpp)
target_include_directories(long_amogus_lib PUBLIC include)
add_library(ducker_publisher_lib STATIC src/ducker_publisher.cpp)
target_include_directories(ducker_publisher_lib PUBLIC include)
target_link_libraries(ducker_publisher_lib long_amogus_lib)
ament_target_dependencies(ducker_publisher_lib rclcpp std_msgs)
add_executable(talker src/ducker_publisher.cpp)
target_compile_definitions(talker PRIVATE DUCKER_STANDALONE)
target_link_libraries(talker long_amogus_lib)
ament_target_dependencies(talker rclcpp std_msgs)

add_library(rick_roll_lib STATIC src/rick_roll.cpp)
target_include_directories(rick_roll_lib PUBLIC include)
add_library(ducker_subscriber_lib STATIC src/ducker_subscriber.cpp)
target_include_directories(ducker_subscriber_lib PUBLIC include)
target_link_libraries(ducker_subscriber_lib rick_roll_lib)
ament_target_dependencies(ducker_subscriber_lib rclcpp std_msgs)
add_executable(listener src/ducker_subscriber.cpp)
target_compile_definitions(listener PRIVATE DUCKER_STANDALONE)
target_link_libraries(listener rick_roll_lib)
ament_target_dependencies(listener rclcpp std_msgs)

install(TARGETS
        talker
        listener
        DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_duck test/test_duck.cpp)
  target_link_libraries(test_duck duck_lib)
  ament_add_gtest(test_long_amogus test/test_long_amogus.cpp)
  target_link_libraries(test_long_amogus long_amogus_lib)
  ament_add_gtest(test_rick_roll test/test_rick_roll.cpp)
  target_link_libraries(test_rick_roll rick_roll_lib)
  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  function(add_ros_isolated_launch_test path)
    set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
    add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
  endfunction()
  add_ros_isolated_launch_test(test/test_ducker_nodes.py)

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
