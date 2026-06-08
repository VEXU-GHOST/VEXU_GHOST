cmake_minimum_required(VERSION 3.8)
project(push_back_cv)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CvDetection.msg"
  "msg/CvDetectionArray.msg"
  "msg/FieldBlock.msg"
  "msg/FieldBlockArray.msg"
  "msg/GoalState.msg"
  "msg/GoalStateArray.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

find_package(ament_cmake_python REQUIRED)
ament_python_install_package(cv_perception)

find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(goal_reader_node src/goal_reader_node.cpp)
ament_target_dependencies(goal_reader_node ament_index_cpp rclcpp)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(goal_reader_node "${cpp_typesupport_target}" yaml-cpp)
install(TARGETS goal_reader_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(
  PROGRAMS
    scripts/cv_detector.py
    scripts/cv_detector_array.py
    scripts/block_map_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
