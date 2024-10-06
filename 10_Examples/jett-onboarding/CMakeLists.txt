cmake_minimum_required(VERSION 3.8)
project(jett-onboarding)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(ament_cmake_gtest REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)
install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

add_executable(hello_world src/hello_world.cpp)
install(TARGETS
  hello_world
  DESTINATION lib/${PROJECT_NAME})
target_link_libraries(hello_world
talking
)

# Add Class as Library to allow it to share across files easier probably
add_library(talking SHARED src/talking.cpp)
ament_target_dependencies(talking
  ${DEPENDENCIES}
)
ament_export_targets(talking HAS_LIBRARY_TARGET)
install(
  TARGETS talking
  EXPORT talking
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
  )


ament_add_gtest(testFile src/testFile.cpp)
target_link_libraries(testFile
  talking
  gtest_main
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)

#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)

#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
